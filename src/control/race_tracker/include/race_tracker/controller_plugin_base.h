#ifndef RACE_TRACKER_CONTROLLER_PLUGIN_BASE_H
#define RACE_TRACKER_CONTROLLER_PLUGIN_BASE_H

#include <ros/ros.h>
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>
#include <string>
#include <race_msgs/Flag.h>

namespace race_tracker {

/**
 * @brief 控制器插件基类接口，所有插件必须继承此接口
 */
class ControllerPluginBase {
public:
    virtual ~ControllerPluginBase() = default; // 虚析构函数，确保子类内存正确释放

    /**
     * @brief 插件初始化（加载参数、初始化成员变量）
     * @param nh 节点句柄（用于加载参数）
     */
    virtual bool initialize(ros::NodeHandle& nh) = 0;

    /**
     * @brief 核心控制计算（插件需实现此方法修改控制指令）
     * @param vehicle_status 最新车辆状态
     * @param path 最新局部路径
     * @param control_msg 待修改的控制指令指针（插件直接修改此对象）
     * @param dt 两次控制的时间间隔（s），用于PID等需要时间差的算法
     */
    virtual void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) = 0;

    /**
     * @brief 获取插件名称（用于日志和调试）
     */
    virtual std::string getName() const = 0;

protected:
    // 工具函数：打印参数加载日志（统一格式）
    void logParamLoad(const std::string& param_name, const double& value, const double& default_val) {
        ROS_INFO("[%s] 加载参数: %s = %.2f (默认值: %.2f)", 
                 getName().c_str(), param_name.c_str(), value, default_val);
    }
};

} // namespace race_tracker

#endif // RACE_TRACKER_CONTROLLER_PLUGIN_BASE_H