#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <race_tracker/controller_plugin_base.h>
#include <race_tracker/pure_pursuit_controller.h>
#include <race_tracker/pid_controller.h>
#include <race_msgs/Control.h>
#include <race_msgs/Path.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Flag.h>
#include <memory>
#include <vector>

namespace race_tracker {

class ControllerManager {
public:
    ControllerManager() : nh_("~"), last_control_time_(ros::Time::now()) {
        // 设置中文编码
        setlocale(LC_ALL, "zh_CN.UTF-8");
        // 1. 加载核心参数（控制频率、话题名称）
        loadCoreParameters();

        // 2. 初始化插件加载器
        initPluginLoader();

        // 3. 加载并初始化插件
        loadAndInitPlugins();

        // 4. 初始化订阅者/发布者（话题名称从参数加载）
        initSubscribersAndPublisher();

        // 5. 初始化默认控制指令
        initDefaultControlMsg();

        // 6. 启动周期控制定时器（核心：固定频率运行）
        startControlTimer();

        ROS_INFO("[ControllerManager] 初始化完成！控制频率: %.1f Hz | 加载插件数: %lu",
                 control_freq_, plugins_.size());
    }

private:
    /**
     * @brief 加载核心参数（控制频率、话题名称、插件列表）
     */
    void loadCoreParameters() {
        // 控制频率（Hz），默认10Hz
        nh_.param("control_freq", control_freq_, 10.0);
        // 话题名称（可通过launch覆盖）
        nh_.param("topic/local_path", local_path_topic_, std::string("/race/local_path"));
        nh_.param("topic/vehicle_state", vehicle_state_topic_, std::string("/race/vehicle_state"));
        nh_.param("topic/control", control_topic_, std::string("/race/control"));
        nh_.param("topic/flag", flag_topic_, std::string("/race/flag"));
        nh_.param("ego_frame_id", ego_frame_id_, std::string("ego_vehicle"));
        // 插件列表（必须在launch中配置）
        if (!nh_.getParam("plugins", plugin_names_)) {
            ROS_FATAL("[ControllerManager] 未配置插件列表（参数名: plugins），请检查launch文件！");
            ros::shutdown();
            return;
        }

        ROS_INFO("[ControllerManager] 核心参数加载完成：");
        ROS_INFO("  - 控制频率: %.1f Hz", control_freq_);
        ROS_INFO("  - 局部路径话题: %s", local_path_topic_.c_str());
        ROS_INFO("  - 车辆状态话题: %s", vehicle_state_topic_.c_str());
        ROS_INFO("  - 控制指令话题: %s", control_topic_.c_str());
        ROS_INFO("  - 标志话题: %s", flag_topic_.c_str());
        ROS_INFO("  - 插件列表: ");
        for (const auto& plugin : plugin_names_) {
            ROS_INFO("    - %s", plugin.c_str());
        }
    }

    /**
     * @brief 初始化插件加载器（pluginlib）
     */
    void initPluginLoader() {
        try {
            plugin_loader_.reset(new pluginlib::ClassLoader<ControllerPluginBase>(
                "race_tracker", // 包名
                "race_tracker::ControllerPluginBase" // 基类名
            ));
        } catch (pluginlib::PluginlibException& ex) {
            ROS_FATAL("[ControllerManager] 插件加载器初始化失败！错误: %s", ex.what());
            ros::shutdown();
            return;
        }
    }

    /**
     * @brief 加载并初始化所有插件
     */
    void loadAndInitPlugins() {
        for (const auto& plugin_name : plugin_names_) {
            try {
                // 动态创建插件实例
                auto plugin_ptr = plugin_loader_->createInstance(plugin_name);
                // 初始化插件（传递节点句柄加载参数）
                if (plugin_ptr->initialize(nh_)) {
                    plugins_.push_back(plugin_ptr);
                    ROS_INFO("[ControllerManager] 成功加载插件: %s（名称: %s）",
                             plugin_name.c_str(), plugin_ptr->getName().c_str());
                } else {
                    ROS_ERROR("[ControllerManager] 插件初始化失败: %s", plugin_name.c_str());
                }
            } catch (pluginlib::PluginlibException& ex) {
                ROS_ERROR("[ControllerManager] 加载插件 %s 失败！错误: %s",
                          plugin_name.c_str(), ex.what());
            }
        }

        // 检查是否加载到有效插件
        if (plugins_.empty()) {
            ROS_FATAL("[ControllerManager] 未加载到任何有效插件，无法运行！");
            ros::shutdown();
            return;
        }
    }

    /**
     * @brief 初始化订阅者和发布者
     */
    void initSubscribersAndPublisher() {
        // 订阅局部路径（队列大小1，确保最新数据）
        path_sub_ = nh_.subscribe(
            local_path_topic_, 1, &ControllerManager::pathCallback, this);
        // 订阅车辆状态（队列大小1）
        status_sub_ = nh_.subscribe(
            vehicle_state_topic_, 1, &ControllerManager::statusCallback, this);
        // 发布控制指令（队列大小1）
        control_pub_ = nh_.advertise<race_msgs::Control>(
            control_topic_, 1);
        // 订阅标志话题（队列大小1）
        flag_sub_ = nh_.subscribe(
            flag_topic_, 1, &ControllerManager::flagCallback, this);
    }

    /**
     * @brief 初始化默认控制指令（避免未初始化字段）
     */
    void initDefaultControlMsg() {
        control_msg_.header.frame_id = ego_frame_id_; // 自车坐标系
        control_msg_.gear = race_msgs::Control::GEAR_1; // 默认1档
        control_msg_.emergency = false; // 非紧急状态
        control_msg_.hand_brake = false; // 手刹关闭
        control_msg_.clutch = false; // 离合关闭
        control_msg_.control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY; // 默认油门刹车模式
        control_msg_.steering_mode = race_msgs::Control::FRONT_STEERING_MODE; // 默认前轮转向
        // 横向/纵向默认值
        control_msg_.lateral.steering_angle = 0.0;
        control_msg_.lateral.steering_angle_velocity = 0.0;
        control_msg_.lateral.rear_wheel_angle = 0.0;
        control_msg_.lateral.rear_wheel_angle_velocity = 0.0;
        control_msg_.longitudinal.velocity = 0.0;
        control_msg_.longitudinal.acceleration = 0.0;
        control_msg_.longitudinal.jerk = 0.0;
        // 油门刹车默认0
        control_msg_.throttle = 0.0;
        control_msg_.brake = 0.0;
    }

    /**
     * @brief 启动周期控制定时器
     */
    void startControlTimer() {
        double control_period = 1.0 / control_freq_; // 控制周期（s）
        control_timer_ = nh_.createTimer(
            ros::Duration(control_period),
            &ControllerManager::controlCycleCallback,
            this);
    }

    /**
     * @brief 路径回调：保存最新路径数据
     */
    void pathCallback(const race_msgs::PathConstPtr& path_msg) {
        std::lock_guard<std::mutex> lock(path_mutex_); // 线程安全
        last_path_ = path_msg;
        // ROS_DEBUG("[ControllerManager] 接收新路径（点数: %lu）", path_msg->points.size());
    }

    /**
     * @brief 车辆状态回调：保存最新车辆状态
     */
    void statusCallback(const race_msgs::VehicleStatusConstPtr& status_msg) {
        std::lock_guard<std::mutex> lock(status_mutex_); // 线程安全
        last_status_ = status_msg;
        // ROS_DEBUG("[ControllerManager] 接收新车辆状态（速度: %.2f m/s）", status_msg->vel.linear.x);
    }

    /**
     * @brief 标志话题回调：保存最新标志数据
     */
    void flagCallback(const race_msgs::Flag::ConstPtr& flag_msg) {
        std::lock_guard<std::mutex> lock(flag_mutex_); // 线程安全
        last_flag_ = flag_msg;
        // ROS_DEBUG("[ControllerManager] 接收新标志（状态: %d）", flag_msg->flag);
    }

    /**
     * @brief 周期控制回调（核心：调用插件计算控制指令）
     */
    void controlCycleCallback(const ros::TimerEvent& event) {
        // 1. 计算控制时间差（dt）
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_control_time_).toSec();
        last_control_time_ = current_time;

        // 2. 检查是否有最新的路径和车辆状态
        race_msgs::PathConstPtr path;
        race_msgs::VehicleStatusConstPtr status;
        {
            std::lock_guard<std::mutex> path_lock(path_mutex_);
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            if (!last_path_ || !last_status_) {
                ROS_WARN_THROTTLE(1.0, "[ControllerManager] 等待路径/车辆状态数据（已等待: %.1f s）",
                                  (current_time - ros::Time::now()).toSec() * (-1));
                return;
            }
            path = last_path_;
            status = last_status_;
        }

        // 3. 重置控制指令（避免历史值干扰，保留基础字段）
        resetControlMsg();

        // 4. 调用所有插件计算控制指令（按插件列表顺序执行，后执行的覆盖前执行的）
        for (const auto& plugin : plugins_) {
            plugin->computeControl(status, path, &control_msg_, dt, last_flag_);
        }

        // 5. 设置控制指令时间戳并发布
        control_msg_.header.stamp = current_time;
        control_pub_.publish(control_msg_);
    }

    /**
     * @brief 重置控制指令（保留基础配置，如档位、模式，重置控制量）
     */
    void resetControlMsg() {
        // 保留基础字段（不重置）
        // control_msg_.gear, control_msg_.control_mode, control_msg_.steering_mode 等
        // 重置控制量字段
        control_msg_.throttle = 0.0;
        control_msg_.brake = 0.0;
        control_msg_.lateral.steering_angle = 0.0;
        control_msg_.lateral.steering_angle_velocity = 0.0;
        control_msg_.lateral.rear_wheel_angle = 0.0;
        control_msg_.lateral.rear_wheel_angle_velocity = 0.0;
        control_msg_.longitudinal.velocity = 0.0;
        control_msg_.longitudinal.acceleration = 0.0;
        control_msg_.longitudinal.jerk = 0.0;
    }

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    // 订阅者/发布者
    ros::Subscriber path_sub_;
    ros::Subscriber status_sub_;
    ros::Publisher control_pub_;
    ros::Subscriber flag_sub_; // 标志话题订阅者
    // 周期控制定时器
    ros::Timer control_timer_;
    // 插件管理
    std::unique_ptr<pluginlib::ClassLoader<ControllerPluginBase>> plugin_loader_;
    std::vector<boost::shared_ptr<ControllerPluginBase>> plugins_;
    std::vector<std::string> plugin_names_;
    // 数据缓存（最新路径和车辆状态）
    race_msgs::PathConstPtr last_path_;
    race_msgs::VehicleStatusConstPtr last_status_;
    race_msgs::Flag::ConstPtr last_flag_; // 标志话题数据
    // 控制指令（持续修改并发布）
    race_msgs::Control control_msg_;
    // 线程安全锁（避免回调与控制周期并发访问数据）
    std::mutex path_mutex_;
    std::mutex status_mutex_;
    std::mutex flag_mutex_; // 标志话题数据锁
    // 核心参数
    double control_freq_; // 控制频率（Hz）
    std::string ego_frame_id_; // ego车辆坐标系
    std::string local_path_topic_; // 局部路径话题
    std::string vehicle_state_topic_; // 车辆状态话题
    std::string control_topic_; // 控制指令话题
    std::string flag_topic_; // 标志话题
    // 时间戳（计算dt）
    ros::Time last_control_time_;
};

} // namespace race_tracker

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_manager");
    race_tracker::ControllerManager cm;
    ros::spin();
    return 0;
}