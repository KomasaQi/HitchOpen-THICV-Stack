#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Control.h>
#include <race_msgs/Euler.h>
#include <race_msgs/Lateral.h>
#include <race_msgs/WheelSpeed.h>
#include <race_msgs/Path.h>


const double PI = 3.1415926; 

class StateConverter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber imu_sub_;
    ros::Subscriber imu_trailer_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber speedometer_sub_;
    ros::Subscriber wheel_rpm_truck_sub_;
    ros::Subscriber path_sub_;



    
    // 发布者
    ros::Publisher vehicle_state_pub_;
    ros::Publisher vehicle_pose_vis_pub_;
    
    // 存储接收到的消息
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::Imu imu_trailer_msg_;
    nav_msgs::Odometry odom_msg_;
    std_msgs::Float32 wheel_rpm_truck_msg_;
    std_msgs::Float32 speedometer_msg_;
    race_msgs::Path path_msg_;



    
    // 消息接收标志
    bool wheel_rpm_truck_received_;
    bool odom_received_;
    bool imu_received_;
    bool imu_trailer_received_;
    bool speedometer_received_;
    bool path_received_;

    
    // 车辆参数（可能需要根据实际车辆调整）
    double wheel_radius_;

    // 不断被修改用于发布的状态
    race_msgs::VehicleStatus state_msg_;
    nav_msgs::Odometry pose_vis_msg_; // 新增：用于可视化车辆状态的位姿消息

    ros::Timer track_timer_;       // 新增：跟踪误差计算计时器
    ros::Timer publish_timer_;     // 新增：状态发布计时器
    
    
public:
    StateConverter() : private_nh_("~"), 
                   wheel_rpm_truck_received_(false),
                   speedometer_received_(false),
                   odom_received_(false),
                   imu_trailer_received_(false),
                   imu_received_(false),
                   path_received_(false) {
        // 获取车辆参数
        private_nh_.param<double>("wheel_radius", wheel_radius_, 0.30);
        
        // 初始化订阅者
        imu_sub_ = nh_.subscribe("/tractor/imu/data", 10, &StateConverter::imuCallback, this);
        imu_trailer_sub_ = nh_.subscribe("/trailer/imu/data", 10, &StateConverter::imuTrailerCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/imu", 10, &StateConverter::odomCallback, this);
        wheel_rpm_truck_sub_ = nh_.subscribe("/race/wheel_rate", 10, &StateConverter::wheelRpmCallback, this);
        speedometer_sub_ = nh_.subscribe("/race/speedometer", 10, &StateConverter::speedometerCallback, this);
        path_sub_ = nh_.subscribe("/race/local_path", 10, &StateConverter::pathCallback, this);



        // 初始化发布者
        vehicle_state_pub_ = nh_.advertise<race_msgs::VehicleStatus>("/race/vehicle_state", 10);
        vehicle_pose_vis_pub_ = nh_.advertise<nav_msgs::Odometry>("/race/pose_vis", 10);
        
        // 修复：计时器赋值给类成员变量（生命周期与类一致）
        track_timer_ = nh_.createTimer(ros::Duration(0.05), &StateConverter::trackCallback, this);
        publish_timer_ = nh_.createTimer(ros::Duration(0.01), &StateConverter::publishVehicleState, this);

        ROS_INFO("Pix Race State Converter initialized");
    }
    

    double distance2D(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    // 计算跟踪误差
    void trackCallback(const ros::TimerEvent& event) {
        // 如果有路径信息，就计算横向误差、速度误差、航向误差，否则设为0
        if (!path_received_) {
            state_msg_.tracking.lateral_tracking_error = 0.0;
            state_msg_.tracking.heading_angle_error = 0.0;
            state_msg_.tracking.velocity_error = 0.0;
            return;
        } 
        // 新增：路径点数量不足2个时，不计算误差（避免越界）
        if (path_msg_.points.size() < 2) {
            ROS_WARN_THROTTLE(1, "Path has less than 2 points, skip tracking error calculation");
            state_msg_.tracking.lateral_tracking_error = 0.0;
            state_msg_.tracking.heading_angle_error = 0.0;
            state_msg_.tracking.velocity_error = 0.0;
            return;
        }
            
        
        // 找到路径中的最近点id
        int nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_msg_.points.size(); ++i) {
            double dist = distance2D(odom_msg_.pose.pose.position, path_msg_.points[i].pose.position);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        // 1. 获取自车当前位置和路径最近点的位置
        geometry_msgs::Point ego_pos = odom_msg_.pose.pose.position;
        geometry_msgs::Point path_point_pos = path_msg_.points[nearest_idx].pose.position;

        // 2. 计算路径最近点的切线方向向量（需处理边界：最后一个点用前一个点的方向）
        geometry_msgs::Vector3 path_tangent;
        if (nearest_idx == path_msg_.points.size() - 1) {
            // 若为路径最后一个点，使用前一个点与当前点构成切线（避免越界）
            path_tangent.x = path_point_pos.x - path_msg_.points[nearest_idx - 1].pose.position.x;
            path_tangent.y = path_point_pos.y - path_msg_.points[nearest_idx - 1].pose.position.y;
        } else {
            // 非最后一个点，使用当前点与下一个点构成切线（更贴合路径趋势）
            path_tangent.x = path_msg_.points[nearest_idx + 1].pose.position.x - path_point_pos.x;
            path_tangent.y = path_msg_.points[nearest_idx + 1].pose.position.y - path_point_pos.y;
        }

        // 3. 归一化切线方向向量（消除距离对方向计算的影响）
        double tangent_len = std::sqrt(path_tangent.x * path_tangent.x + path_tangent.y * path_tangent.y);
        if (tangent_len < 1e-6) { // 避免除以0（路径点重合的极端情况）
            tangent_len = 1e-6;
        }
        path_tangent.x /= tangent_len;
        path_tangent.y /= tangent_len;

        // 4. 计算自车到路径最近点的向量（P_ego - P_path）
        geometry_msgs::Vector3 ego_to_path;
        ego_to_path.x = ego_pos.x - path_point_pos.x;
        ego_to_path.y = ego_pos.y - path_point_pos.y;

        // 5. 计算横向误差（垂直距离）：利用2D叉积计算垂直分量，区分左右偏差
        // 叉积结果符号规则：自车在路径切线左侧为正，右侧为负（符合车辆控制习惯）
        state_msg_.tracking.lateral_tracking_error = ego_to_path.x * path_tangent.y - ego_to_path.y * path_tangent.x;

        // 6. 计算航向误差：自车当前航向角 - 路径最近点的期望航向角（归一化到[-π, π]）
        // 6.1 从路径最近点的四元数中解析期望航向角（yaw角）
        tf2::Quaternion path_quat(
            path_msg_.points[nearest_idx].pose.orientation.x,
            path_msg_.points[nearest_idx].pose.orientation.y,
            path_msg_.points[nearest_idx].pose.orientation.z,
            path_msg_.points[nearest_idx].pose.orientation.w
        );
        tf2::Matrix3x3 path_rot_mat(path_quat);
        double path_yaw; // 路径最近点的期望航向角（rad）
        double path_roll;
        double path_pitch;
        path_rot_mat.getRPY(path_roll, path_pitch, path_yaw);

        // 6.2 自车当前航向角（已通过quaternionToEuler计算，直接复用）
        double ego_yaw = state_msg_.euler.yaw;

        // 6.3 计算航向误差并归一化到[-π, π]（避免误差超过π导致控制反向）
        double heading_error = ego_yaw - path_yaw;
        if (heading_error > M_PI) {
            heading_error -= 2 * M_PI;
        } else if (heading_error < -M_PI) {
            heading_error += 2 * M_PI;
        }
        // 归一化公式：确保误差在[-π, π]范围内
        std::cout << "期望航向角: " << path_yaw << ", 自车航向角: " << ego_yaw << ", 航向误差: " << heading_error << std::endl;
        state_msg_.tracking.heading_angle_error = heading_error;

        // 7. 计算速度误差：自车当前速度 - 路径最近点的期望速度
        state_msg_.tracking.velocity_error = path_msg_.points[nearest_idx].velocity - state_msg_.vel.linear.x;


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

    }
    // 回调函数
    void imuTrailerCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        imu_trailer_msg_ = *msg;
        imu_trailer_received_ = true;

    }
    
    void speedometerCallback(const std_msgs::Float32::ConstPtr& msg) {
        speedometer_msg_ = *msg;
        speedometer_received_ = true;

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_msg_ = *msg;
        odom_received_ = true;

    }
    
    
    void wheelRpmCallback(const std_msgs::Float32::ConstPtr& msg) {
        wheel_rpm_truck_msg_ = *msg;
        wheel_rpm_truck_received_ = true;

    }
 
    
    void pathCallback(const race_msgs::Path::ConstPtr& msg) {
        path_msg_ = *msg;
        path_received_ = true;
    }
    
    // 发布转换后的消息
    void publishVehicleState(const ros::TimerEvent& event) {
        // 等待所有必要的消息都被接收
        if ( !wheel_rpm_truck_received_ || !speedometer_received_ || !odom_received_ ) {
            return;
        }

        // 填充header
        state_msg_.header = odom_msg_.header;
        state_msg_.child_frame_id = "ego_vehicle";
        pose_vis_msg_.header = odom_msg_.header;
        pose_vis_msg_.child_frame_id = odom_msg_.child_frame_id;
        
        // 填充pose
        state_msg_.pose = odom_msg_.pose.pose;
        
        
        // 计算并填充euler角
        state_msg_.euler = quaternionToEuler(odom_msg_.pose.pose.orientation);

        // 实际Pose是上述pose向后挪动0.3m（轴距）
        // 实际Pose是上述pose向后挪动0.3m（轴距）
        state_msg_.pose.position.x -= 0.3 * cos(state_msg_.euler.yaw);
        state_msg_.pose.position.y -= 0.3 * sin(state_msg_.euler.yaw);

        pose_vis_msg_.pose.pose = state_msg_.pose;
        pose_vis_msg_.twist.twist = odom_msg_.twist.twist;
    
        
        // 填充速度信息
        state_msg_.vel.linear.x = speedometer_msg_.data;
        
        // 填充加速度信息（来自IMU）
        state_msg_.acc.linear = imu_msg_.linear_acceleration;
        state_msg_.acc.angular = imu_msg_.angular_velocity;
        
        // 填充转向信息
        state_msg_.lateral.steering_angle = 0.0;
        // 这里假设转向角速度无法直接获取，设置为0或需要额外计算
        state_msg_.lateral.steering_angle_velocity = 0.0;

        
        // 填充车轮速度（假设四轮速度相同，根据车速计算）
        state_msg_.wheel_speed.left_front = wheel_rpm_truck_msg_.data/360*60; // 从deg/s转换为rpm
        state_msg_.wheel_speed.left_rear = wheel_rpm_truck_msg_.data/360*60;
        state_msg_.wheel_speed.right_front = wheel_rpm_truck_msg_.data/360*60;
        state_msg_.wheel_speed.right_rear = wheel_rpm_truck_msg_.data/360*60;
        
        // 填充档位信息
        state_msg_.gear = race_msgs::Control::GEAR_1;
        
        // 设置控制模式（默认设置为油门刹车模式）

        state_msg_.control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY;
 
        state_msg_.throttle_fb = 0.0/100.0;
        state_msg_.brake_fb = 0.0/100.0;
        
        // 填充手刹状态
        state_msg_.hand_brake = false;
        
        // 紧急状态（这里简化处理，根据刹车值判断）
        state_msg_.emergency = false;
        
        // 离合状态（CARLA不直接提供，这里设为true）
        state_msg_.clutch = true;
        
        // 转向模式（默认设为前轮转向）
        state_msg_.steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        
        // 发布消息
        vehicle_state_pub_.publish(state_msg_);
        vehicle_pose_vis_pub_.publish(pose_vis_msg_);
    }
};



int main(int argc, char**argv) {
    ros::init(argc, argv, "state_converter_node");
    StateConverter converter;
    ros::spin();
    return 0;
}
