#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <sensor_msgs/Imu.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Control.h>
#include <race_msgs/Euler.h>
#include <race_msgs/Lateral.h>
#include <race_msgs/WheelSpeed.h>

class StateConverter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber speedometer_sub_;
    ros::Subscriber vehicle_status_sub_;
    
    // 发布者
    ros::Publisher vehicle_state_pub_;
    
    // 存储接收到的消息
    sensor_msgs::Imu imu_msg_;
    nav_msgs::Odometry odom_msg_;
    std_msgs::Float32 speedometer_msg_;
    carla_msgs::CarlaEgoVehicleStatus vehicle_status_msg_;
    
    // 消息接收标志
    bool imu_received_;
    bool odom_received_;
    bool speedometer_received_;
    bool vehicle_status_received_;
    
    // 车辆参数（可能需要根据实际车辆调整）
    double wheel_radius_;
    
public:
    StateConverter() : private_nh_("~"), 
                       imu_received_(false),
                       odom_received_(false),
                       speedometer_received_(false),
                       vehicle_status_received_(false) {
        // 获取车辆参数
        private_nh_.param<double>("wheel_radius", wheel_radius_, 0.37);
        
        // 初始化订阅者
        imu_sub_ = nh_.subscribe("/carla/ego_vehicle/imu", 10, &StateConverter::imuCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/imu", 10, &StateConverter::odomCallback, this);
        speedometer_sub_ = nh_.subscribe("/carla/ego_vehicle/speedometer", 10, &StateConverter::speedometerCallback, this);
        vehicle_status_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &StateConverter::vehicleStatusCallback, this);
        
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
    
    void speedometerCallback(const std_msgs::Float32::ConstPtr& msg) {
        speedometer_msg_ = *msg;
        speedometer_received_ = true;
        publishVehicleState();
    }
    
    void vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg) {
        vehicle_status_msg_ = *msg;
        vehicle_status_received_ = true;
        publishVehicleState();
    }
    
    // 发布转换后的消息
    void publishVehicleState() {
        // 等待所有必要的消息都被接收
        if (!imu_received_ || !odom_received_ || !speedometer_received_ || !vehicle_status_received_) {
            return;
        }
        
        race_msgs::VehicleStatus state_msg;
        
        // 填充header
        state_msg.header = odom_msg_.header;
        state_msg.child_frame_id = "ego_vehicle";
        
        // 填充pose
        state_msg.pose = odom_msg_.pose.pose;
        
        // 计算并填充euler角
        state_msg.euler = quaternionToEuler(odom_msg_.pose.pose.orientation);
        
        // 填充速度信息
        state_msg.vel.linear.x = speedometer_msg_.data;
        state_msg.vel.linear.y = 0.0;
        state_msg.vel.linear.z = 0.0;
        state_msg.vel.angular = imu_msg_.angular_velocity;
        
        // 填充加速度信息（来自IMU）
        state_msg.acc.linear = imu_msg_.linear_acceleration;
        // state_msg.acc.angular = imu_msg_.angular_velocity;
        
        // 填充转向信息
        state_msg.lateral.steering_angle = -vehicle_status_msg_.control.steer;
        // 这里假设转向角速度无法直接获取，设置为0或需要额外计算
        state_msg.lateral.steering_angle_velocity = 0.0;
        // 双轴转向相关信息，CARLA默认可能不提供，设置为0
        state_msg.lateral.rear_wheel_angle = 0.0;
        state_msg.lateral.rear_wheel_angle_velocity = 0.0;
        
        // 填充车轮速度（假设四轮速度相同，根据车速计算）
        // 注意：这里是简化处理，实际应用中可能需要更精确的计算
        double wheel_angular_speed = speedometer_msg_.data / wheel_radius_;
        state_msg.wheel_speed.left_front = wheel_angular_speed;
        state_msg.wheel_speed.left_rear = wheel_angular_speed;
        state_msg.wheel_speed.right_front = wheel_angular_speed;
        state_msg.wheel_speed.right_rear = wheel_angular_speed;
        
        // 填充档位信息
        // CARLA的gear为1表示前进，-1表示倒车，这里映射到自定义的档位
        if (vehicle_status_msg_.control.reverse) {
            state_msg.gear = race_msgs::Control::GEAR_REVERSE;
        } else if (vehicle_status_msg_.control.gear == 0) {
            state_msg.gear = race_msgs::Control::GEAR_NEUTRAL;
        } else {
            state_msg.gear = vehicle_status_msg_.control.gear;
        }
        
        // 设置控制模式（默认设置为油门刹车模式）
        state_msg.control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY;
        
        // 填充手刹状态
        state_msg.hand_brake = vehicle_status_msg_.control.hand_brake;
        
        // 紧急状态（这里简化处理，根据刹车值判断）
        state_msg.emergency = (vehicle_status_msg_.control.brake > 0.98);
        
        // 离合状态（CARLA不直接提供，这里设为true）
        state_msg.clutch = true;
        
        // 转向模式（默认设为前轮转向）
        state_msg.steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        
        // 发布消息
        vehicle_state_pub_.publish(state_msg);
    }
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "state_converter_lio_node");
    StateConverter converter;
    ros::spin();
    return 0;
}
