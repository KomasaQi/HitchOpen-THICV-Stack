#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <race_msgs/LateralLoadTransferStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class LTRSensorNode {
public:
    LTRSensorNode() : nh_("~") {
        // 加载参数，设置默认值
        nh_.param<std::string>("imu_topic", imu_topic_, "/imu_message");
        nh_.param<std::string>("ltr_topic", ltr_topic_, "/race/ltr");
        nh_.param<std::string>("frame_id", frame_id_, "ego_vehicle");
        
        // 悬架参数
        nh_.param<double>("suspension_stiffness", suspension_stiffness_, 20000.0);
        nh_.param<double>("damping_coefficient", damping_coefficient_, 1000.0);
        nh_.param<double>("suspension_nonlinearity", suspension_nonlinearity_, 0.2);
        nh_.param<double>("damping_nonlinearity", damping_nonlinearity_, 0.1);
        
        // 车辆质量参数
        nh_.param<double>("sprung_mass", sprung_mass_, 1200.0);       // 簧载质量，单位：千克
        nh_.param<double>("unsprung_mass", unsprung_mass_, 300.0);     // 簧下质量，单位：千克
        nh_.param<double>("cg_height_sprung", cg_height_sprung_, 0.55); // 簧载质量重心高度，单位：米
        nh_.param<double>("cg_height_unsprung", cg_height_unsprung_, 0.3); // 簧下质量重心高度，单位：米
        
        // 其他车辆参数
        nh_.param<double>("track_width", track_width_, 1.5);          // 轮距，单位：米
        nh_.param<double>("gravity", gravity_, 9.81);                 // 重力加速度，单位：m/s²

        // 计算总质量
        total_mass_ = sprung_mass_ + unsprung_mass_;
        
        // 初始化发布者和订阅者
        ltr_pub_ = nh_.advertise<race_msgs::LateralLoadTransferStamped>(ltr_topic_, 10);
        imu_sub_ = nh_.subscribe(imu_topic_, 10, &LTRSensorNode::imuCallback, this);
        
        last_ltr_time_ = -1.0;
        ltr_rate_cache_ = 0.0;  // 初始化为0.0，用于平滑处理
        ROS_INFO("LTR Sensor Node (Nonlinear) initialized.");
        ROS_INFO("Subscribing to IMU topic: %s", imu_topic_.c_str());
        ROS_INFO("Publishing LTR to topic: %s", ltr_topic_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        // 从IMU消息中获取四元数并转换为欧拉角（滚转角、俯仰角、偏航角）
        tf2::Quaternion quat(
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z,
            imu_msg->orientation.w
        );
        
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        
        // 获取侧倾角速度（绕x轴的角速度）
        double roll_rate = imu_msg->angular_velocity.x;
        
        // 获取横向加速度（y轴方向）
        double lateral_accel = imu_msg->linear_acceleration.y;
        
        // 计算侧向载荷转移率（非线性模型）
        double ltr = calculateNonlinearLTR(roll, roll_rate, lateral_accel);
        
        // 如果还没存储LTR，就跳过一次
        if (last_ltr_time_ > 0.0 ){
            // 计算LTR速率
            double ltr_rate = (ltr - last_ltr_) / (imu_msg->header.stamp.toSec() - last_ltr_time_);
            last_ltr_ = ltr;
            last_ltr_time_ = imu_msg->header.stamp.toSec();
            // 平滑处理LTR速率
            ltr_rate_cache_ = 0.6 * ltr_rate_cache_ + 0.4 * ltr_rate;
            // 发布LTR消息
            race_msgs::LateralLoadTransferStamped ltr_msg;
            ltr_msg.header.stamp = imu_msg->header.stamp;
            ltr_msg.header.frame_id = frame_id_;
            ltr_msg.ltr = static_cast<float>(ltr);
            ltr_msg.ltr_rate = static_cast<float>(ltr_rate_cache_);
            ltr_pub_.publish(ltr_msg);

        }else{
            last_ltr_ = ltr;
            last_ltr_time_ = imu_msg->header.stamp.toSec();
        }
    }
    
    // 非线性模型计算LTR
    double calculateNonlinearLTR(double roll_angle, double roll_rate, double lateral_accel) {

        // 计算悬架变形量
        double suspension_deflection = (track_width_ / 2.0) * sin(roll_angle);
        
        // 计算非线性悬架刚度 (K = K0 + K1 * x²)
        double nonlinear_stiffness = suspension_stiffness_ * 
                                    (1.0 + suspension_nonlinearity_ * pow(suspension_deflection, 2));
        
        // 计算静态分量（使用tan函数，更精确的非线性模型）
        double static_component_sprung = (sprung_mass_ * gravity_ * cg_height_sprung_ * tan(roll_angle)) /
                                        (nonlinear_stiffness * track_width_);
                                        
        double static_component_unsprung = (unsprung_mass_ * gravity_ * cg_height_unsprung_ * tan(roll_angle)) /
                                          (nonlinear_stiffness * track_width_);
                                          
        double static_component = static_component_sprung + static_component_unsprung;
        
        // 计算悬架变形速率
        double deflection_rate = (track_width_ / 2.0) * cos(roll_angle) * roll_rate;
        
        // 计算非线性阻尼系数 (C = C0 + C1 * |v|)
        double nonlinear_damping = damping_coefficient_ * 
                                  (1.0 + damping_nonlinearity_ * fabs(deflection_rate));
        
        // 计算动态分量
        double dynamic_component = (nonlinear_damping * deflection_rate * track_width_) /
                                 (total_mass_ * gravity_);
        
        // 加入横向加速度的非线性影响
        double accel_factor = 1.0 + 0.3 * tanh(lateral_accel / gravity_);
        
        // 合并静态和动态分量，并应用加速度因子
        double ltr_raw = (static_component + dynamic_component) * accel_factor;
        
        // 应用最终的饱和处理，确保值在[-1, 1]范围内
        return std::max(-1.0, std::min(1.0, ltr_raw));
    }
    
    ros::NodeHandle nh_;
    ros::Publisher ltr_pub_;
    ros::Subscriber imu_sub_;
    
    // 配置参数
    std::string imu_topic_;
    std::string ltr_topic_;
    std::string frame_id_;
    
    // 悬架参数
    double suspension_stiffness_;
    double damping_coefficient_;
    double suspension_nonlinearity_;  // 悬架非线性系数
    double damping_nonlinearity_;     // 阻尼非线性系数
    
    // 质量参数
    double sprung_mass_;       // 簧载质量
    double unsprung_mass_;     // 簧下质量
    double total_mass_;        // 总质量
    double cg_height_sprung_;  // 簧载质量重心高度
    double cg_height_unsprung_;// 簧下质量重心高度
    
    // 其他车辆参数
    double track_width_;
    double gravity_;
    double max_roll_angle_;
    
    // 用于计算LTR速率的变量
    double last_ltr_;
    double last_ltr_time_;
    double ltr_rate_cache_;
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "ltr_sensor_node");
    
    LTRSensorNode node;
    
    ros::spin();
    return 0;
}
