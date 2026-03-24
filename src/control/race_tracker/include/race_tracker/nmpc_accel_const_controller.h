#ifndef RACE_TRACKER_NMPC_ACCEL_CONST_CONTROLLER_H
#define RACE_TRACKER_NMPC_ACCEL_CONST_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <memory>

#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>

namespace race_tracker {

class NMPCAccelConstController : public ControllerPluginBase {
public:
    NMPCAccelConstController() = default;
    ~NMPCAccelConstController() override = default;

    bool initialize(ros::NodeHandle& nh) override;

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override { return "NMPCAccelConstController"; }

private:
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(double x0, double y0, const race_msgs::Path& path);

    casadi::DM process_race_path(const race_msgs::Path& input_path,
                                 const std::vector<double>& current_state);

    bool solveNMPC(const std::vector<double>& current_state,
                   double vx_now,
                   const casadi::DM& waypoints,
                   std::vector<double>& control_output);

    std::vector<double> vehicleStatusToStateVector(const race_msgs::VehicleStatus& status);

private:
    int nx_;
    int nu_;
    int N_;
    int Nc_;

    double T_d1_;
    double dt_;
    double L_;
    double g_;
    double ay_max_;

    double delta1_min_;
    double delta1_max_;

    double w_pos_;
    double w_theta_;
    double w_delta1_;
    double w_delta_cmd1_;
    double w_delta_cmd_rate_;
    double w_term_pos_;
    double w_term_theta_;

    std::unique_ptr<casadi::OptiSol> sol_prev_;
    casadi::MX X_;
    casadi::MX U_sparse_;
    casadi::MX x0_;
    casadi::MX vx0_;
    casadi::MX waypoints_;
    casadi::Function f_func_;
    casadi::Opti opti_;

    std::vector<double> last_control_output_;
    bool has_prev_sol_;

    std::vector<int> steps_per_control_;
};

} // namespace race_tracker

#endif // RACE_TRACKER_NMPC_ACCEL_CONST_CONTROLLER_H