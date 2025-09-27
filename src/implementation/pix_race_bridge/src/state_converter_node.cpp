#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pix_driver/BatteryStatusFb.h>
#include <pix_driver/SteerStatusFb.h>
#include <pix_driver/BrakeStatusFb.h>
#include <pix_driver/ChassisWheelAngleFb.h>
#include <pix_driver/ChassisWheelRpmFb.h>
#include <pix_driver/DriveStatusFb.h>
#include <pix_driver/VehicleStatusFb.h>
#include <can_msgs/ecu.h>
#include <sensor_msgs/Imu.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Control.h>
#include <race_msgs/Euler.h>
#include <race_msgs/Lateral.h>
#include <race_msgs/WheelSpeed.h>


const double PI = 3.1415926; 

class StateConverter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;

    ros::Subscriber steer_pix_sub_;
    ros::Subscriber brake_pix_sub_;
    ros::Subscriber drive_pix_sub_;
    // ros::Subscriber power_pix_sub_;
    // ros::Subscriber vehicle_state_pix_sub_;
    ros::Subscriber wheel_rpm_pix_sub_;
    // ros::Subscriber vehicle_work_state_sub_;

    
    // 发布者
    ros::Publisher vehicle_state_pub_;
    
    // 存储接收到的消息
    sensor_msgs::Imu imu_msg_;
    nav_msgs::Odometry odom_msg_;
    // std_msgs::Float32 speedometer_msg_;
    // pix_driver::VehicleStatusFb vehicle_state_pix_msg_;
    pix_driver::ChassisWheelRpmFb wheel_rpm_pix_msg_;
    pix_driver::BrakeStatusFb brake_pix_msg_;
    pix_driver::SteerStatusFb steer_pix_msg_;
    pix_driver::DriveStatusFb drive_pix_msg_;

    
    // 消息接收标志
    bool steer_pix_received_;
    bool brake_pix_received_;
    bool drive_pix_received_;
    bool power_pix_received_;
    bool vehicle_state_pix_received_;
    bool wheel_rpm_pix_received_;
    bool odom_received_;
    bool imu_received_;
    
    // 车辆参数（可能需要根据实际车辆调整）
    double wheel_radius_;
    
public:
    StateConverter() : private_nh_("~"), 
                       wheel_rpm_pix_received_(false) {
        // 获取车辆参数
        private_nh_.param<double>("wheel_radius", wheel_radius_, 0.30);
        
        // 初始化订阅者
        imu_sub_ = nh_.subscribe("/imu_raw", 10, &StateConverter::imuCallback, this);
        // odom_sub_ = nh_.subscribe("/liorf_localization/mapping/odometry", 10, &StateConverter::odomCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/imu", 10, &StateConverter::odomCallback, this);
        // speedometer_sub_ = nh_.subscribe("/carla/ego_vehicle/speedometer", 10, &StateConverter::speedometerCallback, this);
        // vehicle_status_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &StateConverter::vehicleStatusCallback, this);
        wheel_rpm_pix_sub_ = nh_.subscribe("/can/wheel_rpm", 10, &StateConverter::wheelRpmCallback, this);
        brake_pix_sub_ = nh_.subscribe("/can/brake_status", 10, &StateConverter::brakeCallback, this);
        steer_pix_sub_ = nh_.subscribe("/can/steer_status", 10, &StateConverter::steerCallback, this);
        drive_pix_sub_ = nh_.subscribe("/can/drive_status", 10, &StateConverter::driveCallback, this);
        

        // 初始化发布者
        vehicle_state_pub_ = nh_.advertise<race_msgs::VehicleStatus>("/race/vehicle_state", 10);
        

        ROS_INFO("Carla Race State Converter initialized");
    }
    
    // 四元数转欧拉角
    race_msgs::Euler quaternionToEuler(const geometry_msgs::Quaternion& quat) {
        race_msgs::Euler euler;
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(euler.roll, euler.pitch, euler.yaw);
        return euler;
    }
    
    // 回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        imu_msg_ = *msg;
        imu_received_ = true;
        publishVehicleState();
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_msg_ = *msg;
        odom_received_ = true;
        publishVehicleState();
    }
    
    // void speedometerCallback(const std_msgs::Float32::ConstPtr& msg) {
    //     speedometer_msg_ = *msg;
    //     speedometer_received_ = true;
    //     publishVehicleState();
    // }
    
    // void vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg) {
    //     vehicle_status_msg_ = *msg;
    //     vehicle_status_received_ = true;
    //     publishVehicleState();
    // }
    
    void wheelRpmCallback(const pix_driver::ChassisWheelRpmFb::ConstPtr& msg) {
        wheel_rpm_pix_msg_ = *msg;
        wheel_rpm_pix_received_ = true;
        publishVehicleState();
    }
    
    void brakeCallback(const pix_driver::BrakeStatusFb::ConstPtr& msg) {
        brake_pix_msg_ = *msg;
        brake_pix_received_ = true;
        publishVehicleState();
    }
    
    void steerCallback(const pix_driver::SteerStatusFb::ConstPtr& msg) {
        steer_pix_msg_ = *msg;
        steer_pix_received_ = true;
        publishVehicleState();
    }
    
    void driveCallback(const pix_driver::DriveStatusFb::ConstPtr& msg) {
        drive_pix_msg_ = *msg;
        drive_pix_received_ = true;
        publishVehicleState();
    }
    
    // 发布转换后的消息
    void publishVehicleState() {
        // 等待所有必要的消息都被接收
        if ( !wheel_rpm_pix_received_ || !brake_pix_received_ || !steer_pix_received_) {
            return;
        }
        
        race_msgs::VehicleStatus state_msg;
        
        // 填充header
        state_msg.header = wheel_rpm_pix_msg_.header;
        state_msg.child_frame_id = "ego_vehicle";
        
        // 填充pose
        state_msg.pose = odom_msg_.pose.pose;
        
        // 计算并填充euler角
        state_msg.euler = quaternionToEuler(odom_msg_.pose.pose.orientation);
        
        // 填充速度信息
        double wheel_speed = (wheel_rpm_pix_msg_.wheel_rpm_lf + wheel_rpm_pix_msg_.wheel_rpm_rf 
            + wheel_rpm_pix_msg_.wheel_rpm_lr + wheel_rpm_pix_msg_.wheel_rpm_rr) / 4.0 / 60.0 * 2*PI;
        state_msg.vel.linear.x = wheel_speed * wheel_radius_ ;
        
        // 填充加速度信息（来自IMU）
        state_msg.acc.linear = imu_msg_.linear_acceleration;
        state_msg.acc.angular = imu_msg_.angular_velocity;
        
        // 填充转向信息
        state_msg.lateral.steering_angle = -steer_pix_msg_.steer_angle_front/450*23/180*PI;
        // 这里假设转向角速度无法直接获取，设置为0或需要额外计算
        state_msg.lateral.steering_angle_velocity = 0.0;
        // 双轴转向相关信息，CARLA默认可能不提供，设置为0
        state_msg.lateral.rear_wheel_angle = -steer_pix_msg_.steer_angle_rear/450*23/180*PI;
        state_msg.lateral.rear_wheel_angle_velocity = 0.0;
        
        // 填充车轮速度（假设四轮速度相同，根据车速计算）
        // 注意：这里是简化处理，实际应用中可能需要更精确的计算
        // double wheel_angular_speed = speedometer_msg_.data / wheel_radius_;
        state_msg.wheel_speed.left_front = wheel_rpm_pix_msg_.wheel_rpm_lf;
        state_msg.wheel_speed.left_rear = wheel_rpm_pix_msg_.wheel_rpm_lr;
        state_msg.wheel_speed.right_front = wheel_rpm_pix_msg_.wheel_rpm_rf;
        state_msg.wheel_speed.right_rear = wheel_rpm_pix_msg_.wheel_rpm_rr;
        
        // 填充档位信息
        if (drive_pix_msg_.gear_status == pix_driver::DriveStatusFb::GearStatusR) {
            state_msg.gear = race_msgs::Control::GEAR_REVERSE;
        } else if (drive_pix_msg_.gear_status == pix_driver::DriveStatusFb::GearStatusN ){
            state_msg.gear = race_msgs::Control::GEAR_PARK;
        } else if (drive_pix_msg_.gear_status == pix_driver::DriveStatusFb::GearStatusD) {
            state_msg.gear = race_msgs::Control::GEAR_1;
        } else {
            state_msg.gear = drive_pix_msg_.gear_status;
        }
        
        // 设置控制模式（默认设置为油门刹车模式）
        if (drive_pix_msg_.drive_mode == pix_driver::DriveStatusFb::DriveModeSpeedCtrl) {
            state_msg.control_mode = race_msgs::Control::DES_SPEED_ONLY;
        } else {
            state_msg.control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY;
        }
        // state_msg.throttle_fb = ;
        state_msg.brake_fb = brake_pix_msg_.brake_pedal/100.0;
        
        // 填充手刹状态
        state_msg.hand_brake = brake_pix_msg_.epb_status > 0u;
        
        // 紧急状态（这里简化处理，根据刹车值判断）
        state_msg.emergency = brake_pix_msg_.epb_status > 0u;
        
        // 离合状态（CARLA不直接提供，这里设为true）
        state_msg.clutch = true;
        
        // 转向模式（默认设为前轮转向）
        if (steer_pix_msg_.steer_mode == pix_driver::SteerStatusFb::SteerModeFrontAckerman) {
            state_msg.steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        } else if (steer_pix_msg_.steer_mode == pix_driver::SteerStatusFb::SteerModeBackAckerman) {
            state_msg.steering_mode = race_msgs::Control::BACK_STEERING_MODE;
        } else if (steer_pix_msg_.steer_mode == pix_driver::SteerStatusFb::SteerModeFrontDifferentBack) {
            state_msg.steering_mode = race_msgs::Control::CENTER_STEERING_MODE;
        } else if (steer_pix_msg_.steer_mode == pix_driver::SteerStatusFb::SteerModeSameFrontBack) {
            state_msg.steering_mode = race_msgs::Control::WEDGE_STEERING_MODE;
        } else if (steer_pix_msg_.steer_mode == pix_driver::SteerStatusFb::SteerModeFrontBack) {
            state_msg.steering_mode = race_msgs::Control::DUAL_STEERING_MODE;
        } else {
            state_msg.steering_mode = steer_pix_msg_.steer_mode;
        }
        
        // 发布消息
        vehicle_state_pub_.publish(state_msg);
    }
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "state_converter_node");
    StateConverter converter;
    ros::spin();
    return 0;
}
