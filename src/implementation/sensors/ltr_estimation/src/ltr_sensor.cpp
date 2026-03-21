#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <race_msgs/LateralLoadTransferStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <limits>
#include <string>

class LTRSensorNode {
public:
    LTRSensorNode() : nh_("~"),
                      initialized_(false),
                      calibrating_(true),
                      calib_count_(0),
                      calib_roll_sum_(0.0),
                      roll_bias_(0.0),
                      last_time_(-1.0),
                      last_ltr_(0.0),
                      ltr_rate_cache_(0.0) {
        // topics
        nh_.param<std::string>("imu_topic", imu_topic_, "/trailer/imu/data");
        nh_.param<std::string>("ltr_topic", ltr_topic_, "/race/ltr");
        nh_.param<std::string>("frame_id", frame_id_, "imu_link");

        // vehicle parameters
        nh_.param<double>("total_mass", total_mass_, 60.0);      // kg
        nh_.param<double>("track_width", track_width_, 0.18);    // m
        nh_.param<double>("gravity", gravity_, 9.806);           // m/s^2

        // equivalent suspension parameters
        // K_eq unit: N/m  (maps left-right suspension deflection difference -> load difference)
        // C_eq unit: N/(m/s)
        nh_.param<double>("equivalent_stiffness", equivalent_stiffness_, 1800.0);
        nh_.param<double>("equivalent_damping", equivalent_damping_, 120.0);

        // optional nonlinear terms
        nh_.param<double>("stiffness_nonlinearity", stiffness_nonlinearity_, 0.0); // 1/m^2
        nh_.param<double>("damping_nonlinearity", damping_nonlinearity_, 0.0);     // 1/(m/s)

        // startup calibration
        nh_.param<double>("calibration_duration", calibration_duration_, 3.0);      // s
        nh_.param<double>("calibration_max_roll_rate", calibration_max_roll_rate_, 0.05); // rad/s

        // filters and protection
        nh_.param<double>("roll_lpf_alpha", roll_lpf_alpha_, 0.20);
        nh_.param<double>("roll_rate_lpf_alpha", roll_rate_lpf_alpha_, 0.25);
        nh_.param<double>("ltr_rate_lpf_alpha", ltr_rate_lpf_alpha_, 0.80);
        nh_.param<double>("min_dt", min_dt_, 1e-3);

        // sign correction
        nh_.param<double>("sign_correction", sign_correction_, 1.0);

        // optional limit
        nh_.param<double>("max_roll_for_estimation", max_roll_for_estimation_, 0.70); // rad

        ltr_pub_ = nh_.advertise<race_msgs::LateralLoadTransferStamped>(ltr_topic_, 10);
        imu_sub_ = nh_.subscribe(imu_topic_, 50, &LTRSensorNode::imuCallback, this);

        ROS_INFO("LTR Sensor Node initialized.");
        ROS_INFO("imu_topic: %s", imu_topic_.c_str());
        ROS_INFO("ltr_topic: %s", ltr_topic_.c_str());
        ROS_INFO("frame_id: %s", frame_id_.c_str());
        ROS_INFO("mass=%.3f kg, track=%.3f m, K_eq=%.3f N/m, C_eq=%.3f N/(m/s)",
                 total_mass_, track_width_, equivalent_stiffness_, equivalent_damping_);
        ROS_INFO("Startup roll-bias calibration enabled for %.2f s", calibration_duration_);
    }

private:
    static bool isFinite(double x) {
        return std::isfinite(x);
    }

    static double clamp(double x, double low, double high) {
        if (x < low) return low;
        if (x > high) return high;
        return x;
    }

    double lowPass(double prev, double input, double alpha) {
        alpha = clamp(alpha, 0.0, 1.0);
        return alpha * input + (1.0 - alpha) * prev;
    }

    void publishLTR(const ros::Time& stamp, double ltr, double ltr_rate) {
        race_msgs::LateralLoadTransferStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;
        msg.ltr = static_cast<float>(ltr);
        msg.ltr_rate = static_cast<float>(ltr_rate);
        ltr_pub_.publish(msg);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        const double t = imu_msg->header.stamp.toSec();
        if (!isFinite(t) || t <= 0.0) {
            ROS_WARN_THROTTLE(1.0, "Invalid IMU timestamp.");
            return;
        }

        tf2::Quaternion quat(
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z,
            imu_msg->orientation.w
        );

        if (!isFinite(quat.x()) || !isFinite(quat.y()) || !isFinite(quat.z()) || !isFinite(quat.w())) {
            ROS_WARN_THROTTLE(1.0, "Invalid quaternion.");
            return;
        }

        tf2::Matrix3x3 mat(quat);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        mat.getRPY(roll, pitch, yaw);

        double roll_rate = imu_msg->angular_velocity.x;

        if (!isFinite(roll) || !isFinite(roll_rate)) {
            ROS_WARN_THROTTLE(1.0, "Invalid roll or roll_rate.");
            return;
        }

        // initialize filter states
        if (!initialized_) {
            roll_filt_ = roll;
            roll_rate_filt_ = roll_rate;
            calib_start_time_ = t;
            last_time_ = t;
            last_ltr_ = 0.0;
            ltr_rate_cache_ = 0.0;
            initialized_ = true;
        }

        // filtering
        roll_filt_ = lowPass(roll_filt_, roll, roll_lpf_alpha_);
        roll_rate_filt_ = lowPass(roll_rate_filt_, roll_rate, roll_rate_lpf_alpha_);

        // startup roll bias calibration
        if (calibrating_) {
            if (std::fabs(roll_rate_filt_) < calibration_max_roll_rate_) {
                calib_roll_sum_ += roll_filt_;
                calib_count_++;
            }

            const double calib_elapsed = t - calib_start_time_;
            if (calib_elapsed >= calibration_duration_) {
                if (calib_count_ > 0) {
                    roll_bias_ = calib_roll_sum_ / static_cast<double>(calib_count_);
                } else {
                    roll_bias_ = roll_filt_;
                }
                calibrating_ = false;
                ROS_INFO("Roll bias calibration finished. roll_bias = %.6f rad (%.3f deg), samples = %d",
                         roll_bias_, roll_bias_ * 180.0 / M_PI, calib_count_);
            }

            // During calibration publish zero, avoiding false alarms
            publishLTR(imu_msg->header.stamp, 0.0, 0.0);
            last_time_ = t;
            last_ltr_ = 0.0;
            return;
        }

        // effective roll after bias removal
        double roll_eff = roll_filt_ - roll_bias_;
        roll_eff = clamp(roll_eff, -max_roll_for_estimation_, max_roll_for_estimation_);

        double ltr = calculateSuspensionEquivalentLTR(roll_eff, roll_rate_filt_);
        if (!isFinite(ltr)) {
            ROS_WARN_THROTTLE(1.0, "LTR is not finite.");
            return;
        }

        double dt = t - last_time_;
        double ltr_rate = 0.0;

        if (isFinite(dt) && dt > min_dt_) {
            ltr_rate = (ltr - last_ltr_) / dt;
            if (!isFinite(ltr_rate)) {
                ltr_rate = 0.0;
            }

            ltr_rate_cache_ =
                ltr_rate_lpf_alpha_ * ltr_rate_cache_ +
                (1.0 - ltr_rate_lpf_alpha_) * ltr_rate;

            if (!isFinite(ltr_rate_cache_)) {
                ltr_rate_cache_ = 0.0;
            }
        } else {
            ltr_rate = 0.0;
        }

        last_time_ = t;
        last_ltr_ = ltr;

        publishLTR(imu_msg->header.stamp, ltr, ltr_rate_cache_);
    }

    double calculateSuspensionEquivalentLTR(double roll_eff, double roll_rate) {
        // left-right equivalent suspension deflection difference
        // delta_z > 0 means one side compressed while the other side extended
        const double delta_z = track_width_ * std::sin(roll_eff);
        const double delta_z_rate = track_width_ * std::cos(roll_eff) * roll_rate;

        // optional nonlinear equivalent stiffness
        double k_eq = equivalent_stiffness_ *
                      (1.0 + stiffness_nonlinearity_ * delta_z * delta_z);

        // optional nonlinear equivalent damping
        double c_eq = equivalent_damping_ *
                      (1.0 + damping_nonlinearity_ * std::fabs(delta_z_rate));

        if (!isFinite(k_eq) || !isFinite(c_eq)) {
            return 0.0;
        }

        // load transfer difference generated by equivalent suspension force
        const double delta_f_k = k_eq * delta_z;
        const double delta_f_c = c_eq * delta_z_rate;
        const double delta_f_total = delta_f_k + delta_f_c;

        // normalize by total vertical load
        double ltr = sign_correction_ * delta_f_total / (total_mass_ * gravity_);

        if (!isFinite(ltr)) {
            return 0.0;
        }

        return clamp(ltr, -1.0, 1.0);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher ltr_pub_;
    ros::Subscriber imu_sub_;

    std::string imu_topic_;
    std::string ltr_topic_;
    std::string frame_id_;

    // vehicle parameters
    double total_mass_;
    double track_width_;
    double gravity_;

    // equivalent suspension parameters
    double equivalent_stiffness_;
    double equivalent_damping_;
    double stiffness_nonlinearity_;
    double damping_nonlinearity_;

    // startup calibration
    double calibration_duration_;
    double calibration_max_roll_rate_;

    // filters and protections
    double roll_lpf_alpha_;
    double roll_rate_lpf_alpha_;
    double ltr_rate_lpf_alpha_;
    double min_dt_;
    double sign_correction_;
    double max_roll_for_estimation_;

    // runtime states
    bool initialized_;
    bool calibrating_;
    double calib_start_time_;
    int calib_count_;
    double calib_roll_sum_;
    double roll_bias_;

    double roll_filt_;
    double roll_rate_filt_;

    double last_time_;
    double last_ltr_;
    double ltr_rate_cache_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ltr_sensor_node");
    LTRSensorNode node;
    ros::spin();
    return 0;
}