#ifndef RACE_GLOBAL_STATIC_PLANNER_H
#define RACE_GLOBAL_STATIC_PLANNER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <race_msgs/Path.h>
#include <race_msgs/PathPoint.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Euler.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "curvature.h"
#include "angle_norm.h"

class RaceGlobalStaticPlanner
{
public:
    /**
     * @brief 构造函数：初始化参数、读取CSV路径、创建发布/订阅者
     */
    RaceGlobalStaticPlanner();

    /**
     * @brief 析构函数：默认空实现
     */
    ~RaceGlobalStaticPlanner() = default;

private:
    /**
     * @brief 从CSV文件读取全局路径数据
     * @return 读取成功返回true，失败返回false
     */
    bool readGlobalPathFromCSV();

    /**
     * @brief 预计算全局路径点的累积距离（沿路径的累加距离）
     */
    void precomputePathDistances();

    /**
     * @brief 发布全局路径（定时器回调函数）
     * @param event ROS定时器事件
     */
    void publishGlobalPath(const ros::TimerEvent& event);

    /**
     * @brief 发布局部路径（定时器回调函数）
     * @param event ROS定时器事件
     */
    void publishLocalPath(const ros::TimerEvent& event);


    /**
     * @brief 车辆状态回调：接收车辆位置/航向，生成并发布局部路径
     * @param msg 车辆状态消息（ENU坐标系）
     */
    void vehicleStatusCallback(const race_msgs::VehicleStatus::ConstPtr& msg);

    /**
     * @brief 根据当前车辆状态生成局部路径
     * @param status 当前车辆状态
     * @return 生成的局部路径（race_msgs::Path）
     */
    race_msgs::Path generateLocalPath(const race_msgs::VehicleStatus& status);

    /**
     * @brief 转换race_msgs::Path到nav_msgs::Path（用于RViz可视化）
     * @param race_path 输入的race格式路径
     * @return 转换后的nav格式路径
     */
    nav_msgs::Path convertToNavPath(const race_msgs::Path& race_path);

    // ROS节点句柄
    ros::NodeHandle nh_;          // 全局节点句柄
    ros::NodeHandle pnh_;         // 私有节点句柄（用于参数读取）

    // TF监听器（用于姿态转换）
    tf::TransformListener tf_listener_;

    // 全局路径数据
    std::vector<race_msgs::PathPoint> global_path_points_;  // 全局路径点集合
    std::vector<double> global_path_cumulative_dist_;       // 路径点累积距离（到起点的距离）
    double origin_lat_;  // ENU原点纬度（WGS84，从CSV元数据读取）
    double origin_lon_;  // ENU原点经度（WGS84，从CSV元数据读取）
    double origin_alt_;  // ENU原点海拔（WGS84，从CSV元数据读取）

    // 配置参数（从params.yaml读取）
    std::string csv_file_path_;          // CSV路径文件路径
    double global_pub_rate_;             // 全局路径发布频率（Hz）
    double local_pub_rate_;              // 局部路径发布频率（Hz）
    bool global_vis_enable_;             // 是否启用全局路径可视化
    bool local_vis_enable_;              // 是否启用局部路径可视化
    double local_forward_dist_;          // 局部路径向前覆盖距离（米）
    double local_backward_dist_;         // 局部路径向后覆盖距离（米）
    double weight_x_;                    // 位置距离权重（X轴）
    double weight_y_;                    // 位置距离权重（Y轴）
    double weight_z_;                    // 位置距离权重（Z轴）
    double weight_heading_;              // 航向角距离权重
    
    bool enable_closed_loop_;           // 是否处理为闭环路径

    std::string map_frame_;              // 地图坐标系（如"map"）
    std::string vehicle_status_topic_;  // 车辆状态话题（如"/race/vehicle_status"）
    std::string global_path_topic_;     // 全局路径话题（如"/race/global_path"）
    std::string local_path_topic_;      // 局部路径话题（如"/race/local_path"）
    std::string global_path_vis_topic_; // 全局路径可视化话题（如"/race/global_path_vis"）
    std::string local_path_vis_topic_;  // 局部路径可视化话题（如"/race/local_path_vis"）

    // ROS发布者
    ros::Publisher global_path_pub_;     // 全局路径发布者（/race/global_path）
    ros::Publisher global_path_vis_pub_; // 全局路径可视化发布者（/race/global_path_vis）
    ros::Publisher local_path_pub_;      // 局部路径发布者（/race/local_path）
    ros::Publisher local_path_vis_pub_;  // 局部路径可视化发布者（/race/local_path_vis）

    // ROS订阅者
    ros::Subscriber vehicle_status_sub_; // 车辆状态订阅者（/race/vehicle_status）

    // ROS定时器
    ros::Timer global_pub_timer_;        // 全局路径发布定时器
    ros::Timer local_pub_timer_;         // 局部路径发布定时器

    // 车辆状态缓存
    race_msgs::VehicleStatus current_vehicle_status_; // 当前车辆状态缓存
    bool is_vehicle_status_received_;                 // 车辆状态是否已接收
};

#endif // RACE_GLOBAL_STATIC_PLANNER_H