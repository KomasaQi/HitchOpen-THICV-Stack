#ifndef COMPETITION_TIMER_H
#define COMPETITION_TIMER_H

// ROS核心头文件
#include <ros/ros.h>
#include <ros/param.h>
// 自定义消息头文件
#include <race_msgs/Flag.h>
#include <race_msgs/VehicleStatus.h>
// 标准消息头文件
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
// 坐标变换相关
#include <tf/transform_datatypes.h>
// 标准库头文件
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <map>
#include <string>

class CompetitionTimer {
public:
    // 构造函数
    CompetitionTimer();

    // 初始化旗帜名称-数值映射
    void initFlagMappings();

    // 从参数服务器加载配置（话题名、阈值等）
    void loadParameters();

    // 车辆状态订阅回调函数
    void vehicleStatusCallback(const race_msgs::VehicleStatus::ConstPtr& msg);

    // 定时器回调函数（发布旗帜、检查参数更新）
    void timerCallback(const ros::TimerEvent& event);

    // 检查参数服务器中旗帜状态的更新
    void checkFlagParamUpdate();

    // 完成一圈的逻辑（记录圈时、更新总时间）
    void completeLap();

    // 结束比赛（切换黑旗、计算总时间）
    void finishRace();

    // 计算两点间欧氏距离
    double calculateDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

    // 检查旗帜值是否有效
    bool isValidFlag(uint8_t flag);

    // 检查是否为绿色系旗帜（含G5/G10等）
    bool isGreenFlag(uint8_t flag);

    // 根据旗帜值获取名称（如RED、GREEN）
    std::string getFlagName(uint8_t flag);

    // 根据旗帜名称获取值（如"GREEN"对应race_msgs::Flag::GREEN）
    uint8_t getFlagValue(const std::string& name);

    // 格式化时间（如"1m 23.456s"）
    std::string formatDuration(const ros::Duration& duration);

    // 发布比赛信息到/competition_info话题
    void publishInfo(const std::string& message);

 

    // 节点运行（阻塞等待回调）
    void run();

private:
    // ROS节点句柄（公共同步、私有参数）
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    // ROS发布者/订阅者/定时器
    ros::Publisher flag_pub;       // 发布比赛旗帜
    ros::Publisher info_pub;       // 发布比赛信息
    ros::Subscriber vehicle_status_sub; // 订阅车辆状态
    ros::Timer timer;              // 定时回调器

    // 话题名称（从参数服务器加载，默认有 fallback）
    std::string flag_topic_name;
    std::string info_topic_name;
    std::string vehicle_status_topic_name;

    // 旗帜名称-数值双向映射（用于参数解析和日志打印）
    std::map<std::string, uint8_t> flag_name_to_value;
    std::map<uint8_t, std::string> flag_value_to_name;

    // 旗帜状态（当前/上一帧）
    uint8_t current_flag;
    uint8_t previous_flag;

    // 位置信息（当前位置、起点、终点）
    geometry_msgs::Point current_position_;
    geometry_msgs::Point start_point_;
    geometry_msgs::Point end_point_;

    // 计时信息（比赛开始时间、当前圈开始时间、当前圈数、总圈数）
    ros::Time start_time;
    ros::Time lap_start_time;
    int current_lap;
    int total_laps;

    // 比赛状态标志位
    bool is_closed_loop;          // 是否为闭环赛道（绕圈）
    uint8_t race_state;           // 比赛状态（0: 开始前, 1: 起点内, 2: 圈中, 3: 比赛结束）
    uint8_t BEFORE_START = 0u;
    uint8_t IN_START = 1u;
    uint8_t IN_LAP = 2u;
    uint8_t FINISHED = 3u;


    // 配置参数（从参数服务器加载）
    double publish_rate;               // 信息发布频率（Hz）
    double start_detection_distance;   // 起点检测距离阈值（m）
    double end_detection_distance;     // 终点检测距离阈值（m）
    double distance_threshold_to_leave_start; // 离开起点的距离阈值（避免重复计数）
};

#endif // COMPETITION_TIMER_H
