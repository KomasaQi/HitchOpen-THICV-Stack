#!/usr/bin/env python3
#
# Copyright (c) 2025-2026 Komasa Qi THICV
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
CARLA bridge with steering dynamics identified from semi-trailer road-test data.

Model chain:
    race_msgs/Control.lateral.steering_angle [front-wheel rad]
        -> pure transport delay
        -> steering ratio (front wheel -> steering wheel)
        -> first-order steering response
        -> steering-wheel rate saturation
        -> front-wheel angle
        -> CARLA normalized steer command

Default steering parameters are based on two 20 Hz road-test bags:
  - steering ratio ~= 16.0
  - effective transport delay ~= 0.05~0.09 s
  - dominant first-order time constant ~= 0.08~0.13 s
  - steering-wheel physical rate saturation ~= 2.0 rad/s in the sharp-turn test
  - command-side steering-wheel speed cap = 200 deg/s = 3.490659 rad/s

The old bridge files are intentionally left unchanged. This node is an A/B-testable
third implementation intended to reproduce the real steering actuator more closely.
"""

from collections import deque
import math
import sys

import rospy
from rospy.exceptions import ROSException
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo
from race_msgs.msg import Control


class ControlToVehicleControlRealSteerDynamics:
    """Convert race_msgs/Control to CARLA control with identified steering dynamics."""

    def __init__(self):
        rospy.init_node("race_msgs_to_control_real_steer_dynamics")
        rospy.logwarn(
            "race_msgs_to_control_real_steer_dynamics namespace: %s",
            rospy.get_namespace(),
        )

        self.role_name = rospy.get_param("~role_name", "ego_vehicle")

        # ---- Steering model identified from road-test data ----
        self.steering_ratio = float(rospy.get_param("~steering_ratio", 16.0))
        self.steer_delay = float(rospy.get_param("~steer_delay", 0.06))
        self.steer_tau = float(rospy.get_param("~steer_tau", 0.10))

        # Physical steering-wheel rate limit observed in the sharp-turn data.
        self.steer_wheel_rate_limit = float(
            rospy.get_param("~steer_wheel_rate_limit", 2.0)
        )
        # Command-side cap seen in the real chassis command: 200 deg/s.
        self.steer_wheel_cmd_rate_cap = float(
            rospy.get_param("~steer_wheel_cmd_rate_cap", 3.490659)
        )
        self.honor_input_steer_velocity = bool(
            rospy.get_param("~honor_input_steer_velocity", True)
        )

        # CARLA receives normalized steer [-1, 1], whereas race_msgs/Lateral is rad.
        self.normalize_steer_by_vehicle_max_angle = bool(
            rospy.get_param("~normalize_steer_by_vehicle_max_angle", True)
        )
        self.max_steer_angle_unit = str(
            rospy.get_param("~max_steer_angle_unit", "auto")
        ).lower()

        # ---- Longitudinal dynamics: retained from the previous with_dynamics node ----
        self.acc_delay = float(rospy.get_param("~acc_Lag", 0.05))
        self.brake_delay = float(rospy.get_param("~brake_Lag", 0.05))
        self.acc_tau = float(rospy.get_param("~acc_tau", 0.05))
        self.brake_tau = float(rospy.get_param("~brake_tau", 0.05))

        self.nominal_dt = float(rospy.get_param("~nominal_dt", 0.05))
        self.max_dt = float(rospy.get_param("~max_dt", 0.20))
        self.history_margin = float(rospy.get_param("~history_margin", 0.50))

        if self.steering_ratio <= 0.0:
            raise ValueError("~steering_ratio must be > 0")
        if self.steer_tau <= 0.0:
            raise ValueError("~steer_tau must be > 0")
        if self.steer_wheel_rate_limit <= 0.0:
            raise ValueError("~steer_wheel_rate_limit must be > 0")

        self.max_front_wheel_angle_rad = None
        self.actual_steer_wheel = 0.0
        self.actual_acc = 0.0
        self.actual_brake = 0.0
        self.last_update_time = None

        # Each entry: (time, steer_front_rad, steer_front_rate_rad_s, throttle, brake)
        self.cmd_history = deque()

        self.vehicle_info_sub = rospy.Subscriber(
            "/carla/{}/vehicle_info".format(self.role_name),
            CarlaEgoVehicleInfo,
            self.update_vehicle_info,
            queue_size=1,
        )
        self.race_control_sub = rospy.Subscriber(
            "/race/control",
            Control,
            self.race_control_received,
            queue_size=10,
        )
        self.control_pub = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd".format(self.role_name),
            CarlaEgoVehicleControl,
            queue_size=10,
        )

    def update_vehicle_info(self, vehicle_info):
        if not vehicle_info.wheels:
            rospy.logerr("No wheel info received; cannot determine CARLA max steer angle")
            return

        raw = float(vehicle_info.wheels[0].max_steer_angle)
        if raw <= 0.0:
            rospy.logerr("Invalid CARLA max steer angle: %s", raw)
            return

        unit = self.max_steer_angle_unit
        detected_unit = unit
        if unit == "deg":
            angle_rad = math.radians(raw)
        elif unit == "rad":
            angle_rad = raw
        elif unit == "auto":
            # /carla/.../vehicle_info may expose max_steer_angle in either
            # degrees or radians depending on bridge/version. Detect by range.
            if raw > math.pi:
                detected_unit = "deg(auto)"
                angle_rad = math.radians(raw)
            else:
                detected_unit = "rad(auto)"
                angle_rad = raw
        else:
            rospy.logerr("Unknown ~max_steer_angle_unit=%s; use deg/rad/auto", unit)
            return

        if (not math.isfinite(angle_rad)) or angle_rad < 0.05 or angle_rad > 2.0:
            rospy.logerr(
                "Implausible CARLA max front-wheel steer: raw=%.6f, detected=%s, "
                "converted=%.6f rad. Check /carla/%s/vehicle_info.",
                raw, detected_unit, angle_rad, self.role_name,
            )
            return

        self.max_front_wheel_angle_rad = angle_rad
        rospy.logwarn_once(
            "CARLA max front-wheel steer: raw=%.6f, detected=%s, "
            "using %.6f rad (%.2f deg)",
            raw,
            detected_unit,
            angle_rad,
            math.degrees(angle_rad),
        )

    @staticmethod
    def _clip(value, lo, hi):
        return max(lo, min(hi, value))

    @staticmethod
    def _first_order_exact(state, target, dt, tau):
        """Exact discrete update of x_dot=(target-x)/tau for a zero-order hold."""
        if tau <= 1e-6:
            return target
        alpha = 1.0 - math.exp(-max(0.0, dt) / tau)
        return state + alpha * (target - state)

    def _append_history(self, now, race_control):
        self.cmd_history.append(
            (
                now,
                float(race_control.lateral.steering_angle),
                float(race_control.lateral.steering_angle_velocity),
                float(race_control.throttle),
                float(race_control.brake),
            )
        )

        max_delay = max(self.steer_delay, self.acc_delay, self.brake_delay)
        keep_after = now - max_delay - self.history_margin
        while len(self.cmd_history) > 2 and self.cmd_history[1][0] < keep_after:
            self.cmd_history.popleft()

    def _interp_history(self, query_time, value_index):
        """Linearly interpolate one field in the small command history deque."""
        if not self.cmd_history:
            return 0.0
        if query_time <= self.cmd_history[0][0]:
            return self.cmd_history[0][value_index]
        if query_time >= self.cmd_history[-1][0]:
            return self.cmd_history[-1][value_index]

        prev = self.cmd_history[0]
        for curr in list(self.cmd_history)[1:]:
            if curr[0] >= query_time:
                dt = curr[0] - prev[0]
                if dt <= 1e-9:
                    return curr[value_index]
                r = (query_time - prev[0]) / dt
                return prev[value_index] + r * (
                    curr[value_index] - prev[value_index]
                )
            prev = curr
        return self.cmd_history[-1][value_index]

    def _update_steering(self, now, dt):
        target_front = self._interp_history(now - self.steer_delay, 1)
        input_front_rate = abs(self._interp_history(now - self.steer_delay, 2))
        target_wheel = target_front * self.steering_ratio

        rate_limit = min(
            self.steer_wheel_rate_limit,
            self.steer_wheel_cmd_rate_cap,
        )
        if self.honor_input_steer_velocity and input_front_rate > 1e-6:
            rate_limit = min(rate_limit, input_front_rate * self.steering_ratio)

        desired_rate = (target_wheel - self.actual_steer_wheel) / self.steer_tau
        applied_rate = self._clip(desired_rate, -rate_limit, rate_limit)

        next_wheel = self.actual_steer_wheel + applied_rate * dt
        # Prevent a coarse timestep from stepping past the target.
        if (target_wheel - self.actual_steer_wheel) * (target_wheel - next_wheel) < 0.0:
            next_wheel = target_wheel
        self.actual_steer_wheel = next_wheel

        return target_wheel, applied_rate, rate_limit

    def race_control_received(self, race_control):
        if self.max_front_wheel_angle_rad is None:
            rospy.logwarn_throttle(2.0, "Waiting for CARLA vehicle_info...")
            return

        now = rospy.get_time()
        self._append_history(now, race_control)

        if self.last_update_time is None:
            dt = self.nominal_dt
        else:
            dt = now - self.last_update_time
            if dt <= 0.0 or dt > self.max_dt:
                rospy.logwarn_throttle(
                    2.0,
                    "Unexpected control dt=%.4f s; using nominal_dt=%.4f s",
                    dt,
                    self.nominal_dt,
                )
                dt = self.nominal_dt
        self.last_update_time = now

        target_wheel, wheel_rate, rate_limit = self._update_steering(now, dt)

        acc_cmd = self._interp_history(now - self.acc_delay, 3)
        brake_cmd = self._interp_history(now - self.brake_delay, 4)
        self.actual_acc = self._first_order_exact(
            self.actual_acc, acc_cmd, dt, self.acc_tau
        )
        self.actual_brake = self._first_order_exact(
            self.actual_brake, brake_cmd, dt, self.brake_tau
        )

        actual_front = self.actual_steer_wheel / self.steering_ratio
        if self.normalize_steer_by_vehicle_max_angle:
            steer_out = -actual_front / self.max_front_wheel_angle_rad
        else:
            # Legacy A/B mode: preserves the old bridge's direct-radian-to-CARLA
            # mapping (dimensionally inconsistent, but useful for comparison).
            steer_out = -actual_front
        steer_out = self._clip(steer_out, -1.0, 1.0)

        control = CarlaEgoVehicleControl()
        control.throttle = float(self._clip(self.actual_acc, 0.0, 1.0))
        control.brake = float(self._clip(self.actual_brake, 0.0, 1.0))
        control.steer = float(steer_out)
        control.reverse = race_control.gear == Control.GEAR_REVERSE
        control.hand_brake = race_control.hand_brake

        if race_control.emergency:
            control.throttle = 0.0
            control.brake = 1.0
            control.steer = 0.0
            self.actual_steer_wheel = 0.0

        rospy.loginfo_throttle(
            1.0,
            (
                "real-steer dyn: targetSW=%.3f actualSW=%.3f rad | "
                "front=%.4f rad | rateSW=%.3f/%.3f rad/s | "
                "maxFront=%.4f rad | CARLAsteer=%.4f"
            ),
            target_wheel,
            self.actual_steer_wheel,
            actual_front,
            wheel_rate,
            rate_limit,
            self.max_front_wheel_angle_rad,
            control.steer,
        )

        try:
            self.control_pub.publish(control)
        except ROSException as exc:
            if not rospy.is_shutdown():
                rospy.logwarn("Failed to publish vehicle control: %s", exc)


def main():
    try:
        ControlToVehicleControlRealSteerDynamics()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    except Exception as exc:
        rospy.logerr("real_steer_dynamics node failed: %s", exc)
        raise
    finally:
        rospy.loginfo("real_steer_dynamics node exits")


if __name__ == "__main__":
    main()