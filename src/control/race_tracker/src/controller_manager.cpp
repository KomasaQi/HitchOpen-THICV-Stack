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

        ROS_INFO("[ControllerManager] 初始化完成！控制频率: %.1f Hz | 加载插件数: %lu | 标志检查频率: %.1f Hz",
                 control_freq_, plugins_.size(), flag_check_freq_);
    }

private:
    /**
     * @brief 加载核心参数（控制频率、话题名称、插件列表）
     */
    void loadCoreParameters() {
        // 控制频率（Hz），默认10Hz
        nh_.param("control_freq", control_freq_, 10.0);
        // 标志检查频率（Hz），默认5Hz
        nh_.param("flag_check_freq", flag_check_freq_, 5.0);
        // 话题名称（可通过launch覆盖）
        nh_.param("topic/local_path", local_path_topic_, std::string("/race/local_path"));
        nh_.param("topic/vehicle_state", vehicle_state_topic_, std::string("/race/vehicle_state"));
        nh_.param("topic/control", control_topic_, std::string("/race/control"));
        nh_.param("topic/flag", flag_topic_, std::string("/race/flag"));
        nh_.param("ego_frame_id", ego_frame_id_, std::string("ego_vehicle"));
        nh_.param("dual_steer_enable",dual_axis_steer_, false);
        // 横向误差和航向误差开始降速的阈值
        nh_.param("lateral_error_threshold", lateral_error_threshold_, 0.6);
        nh_.param("heading_error_threshold", heading_error_threshold_, 0.3);
        // 在各种绿色旗帜下因航向误差或横向误差而降低到的最小目标速度（km/h）
        nh_.param("default_min_target_speed", default_min_target_speed_, 5.0);
        // 横向误差和航向误差超出阈值时，目标速度的降低比例
        nh_.param("lateral_error_decrease_ratio", lateral_error_decrease_ratio_, 0.8);
        nh_.param("heading_error_decrease_ratio", heading_error_decrease_ratio_, 1.0);
        
        // 标志超时时间（s），默认1秒
        nh_.param("flag_timeout", flag_timeout_, 1.0);
        // 初始标志状态，默认RED
        init_flag_msg_.flag = (uint8_t)race_msgs::Flag::RED;
        last_flag_ = boost::make_shared<const race_msgs::Flag>(init_flag_msg_);
        time_last_flag_check_ = ros::Time::now().toSec(); // 初始化标志位检查时间


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
        // 速度限制默认为150km/h
        nh_.param("speed_limit", default_speed_limit_, 150.0);
        ROS_INFO("  - 速度限制: %.1f km/h", default_speed_limit_);
        speed_limit_ = default_speed_limit_; // 赋初值
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
        if (dual_axis_steer_){
            control_msg_.steering_mode = race_msgs::Control::DUAL_STEERING_MODE; // 可以设置成双轴转向
        }else {
            control_msg_.steering_mode = race_msgs::Control::FRONT_STEERING_MODE; // 默认前轮转向
        }
        
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
        double flag_period = 1.0 / flag_check_freq_; // 标志检查周期（s）
        control_timer_ = nh_.createTimer(
            ros::Duration(control_period),
            &ControllerManager::controlCycleCallback,
            this);
        flag_timer_ = nh_.createTimer(
            ros::Duration(flag_period),
            &ControllerManager::flagCycleCallback,
            this);
    }

    /**
     * @brief 路径回调：保存最新路径数据,并根据限速重新修改参考速度
     */
    void pathCallback(const race_msgs::PathConstPtr& path_msg) {
        std::lock_guard<std::mutex> lock(path_mutex_); // 线程安全
        // 1. 创建原路径的非const副本（解除只读限制）
        race_msgs::PathPtr new_path(new race_msgs::Path(*path_msg));
        
        // 2. 遍历副本中的路径点，修改速度（不超过限速）
        for (auto& point : new_path->points) { // 此时point是可修改的
            if (point.velocity > speed_limit_) {
                point.velocity = speed_limit_;
            }
        }
        
        // 3. 保存修改后的副本
        last_path_ = new_path;
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
        if (flag_msg->flag != last_flag_->flag) {
            ROS_INFO("[ControllerManager] 标志状态更新（前: %d, 后: %d）",
                     last_flag_->flag, flag_msg->flag);
        }
        last_flag_ = flag_msg;
        time_last_flag_check_ = ros::Time::now().toSec(); // 更新上次检查时间
        // 检查标志状态是否为RED
        if (flag_msg->flag == race_msgs::Flag::RED){
            // 若为RED，设置速度限制为0
            speed_limit_ = 0.0;
            ROS_WARN("\033[31m[ControllerManager] 检测到RED标志，速度限制已设置为0\033[0m");
        } else if (flag_msg->flag == race_msgs::Flag::BLACK){// 检查标志状态是否为BLACK
            // 若为BLACK，设置速度限制为0
            speed_limit_ = 0.0;
            ROS_WARN("\032[31m[ControllerManager] 检测到BLACK标志,\033[36m比赛已经完成!\033[31m速度限制已设置为0\033[0m");
        } else if (last_flag_->flag > 3u){ // 如果是带限速的旗帜
            // 若为带限速的旗帜，设置速度限制为限速值
            speed_limit_ = static_cast<double>(last_flag_->flag)/3.6;
            ROS_WARN("\033[32m[ControllerManager] 检测到带限速的GREEN旗帜，速度限制已设置为 %.1f km/h\033[0m", static_cast<double>(last_flag_->flag));
        }else {
            // 否则，恢复默认速度限制
            speed_limit_ = default_speed_limit_;

            // 自定义ROS_WARN的颜色（例如：改为紫色）
            ROS_WARN("\033[32m检测到GREEN旗帜比赛正常无限速\033[0m");
        }

        // 检测横向误差和航向误差是否超过阈值，若超过就会按照绝对值超出比例降低参考速度
        double lateral_error_abs = std::max(std::abs(last_status_->tracking.lateral_tracking_error) - lateral_error_threshold_,0.0);
        double heading_error_abs = std::max(std::abs(last_status_->tracking.heading_angle_error) - heading_error_threshold_,0.0);
        double target_speed_after_lat_dec = speed_limit_ * (1.0 - lateral_error_abs / lateral_error_threshold_);
        double target_speed_after_head_dec = speed_limit_ * (1.0 - heading_error_abs / heading_error_threshold_);
        // 取横向和航向误差降低后的较小值作为最终目标速度
        double target_speed = std::min(target_speed_after_lat_dec, target_speed_after_head_dec);
        // 取最终目标速度和默认最小目标速度的较大值作为最终目标速度
        target_speed = std::max(target_speed, default_min_target_speed_/3.6);
        if (target_speed < speed_limit_) {
            speed_limit_ = target_speed;
            ROS_WARN("\033[33m[ControllerManager] 横向误差或航向误差高于阈值，速度限制已更新为 %.2f km/h\033[0m", speed_limit_*3.6);
        }




    }
    /**
     * @brief 紧急停止函数
     * @details 当检测到RED标志时，调用此函数设置控制指令为紧急停止
     */
    void emergencyStop() {
        control_msg_.emergency = true;
        control_msg_.throttle = 0.0;
        control_msg_.brake = 1.0;
        ROS_WARN("\033[31m[ControllerManager] ***********紧急停止已触发***********\033[0m");
    }

    /**
     * @brief 周期标志检查回调（检查标志状态是否更新）
     */
    void flagCycleCallback(const ros::TimerEvent& event) {
        // 检查是否有最新的标志数据
        auto wait_time = ros::Time::now().toSec() - time_last_flag_check_;
        if (wait_time > flag_timeout_) {
            ROS_WARN_STREAM("[ControllerManager] 等待标志数据(已等待:" << wait_time <<" s),\033[31m设置当前标志为RED\033[0m");
            // 设置当前标志为RED
            last_flag_ = boost::make_shared<const race_msgs::Flag>(init_flag_msg_);
        }
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
        // 检查标志状态是否为RED或者BLACK
        if (last_flag_->flag == race_msgs::Flag::RED || last_flag_->flag == race_msgs::Flag::BLACK) {
            // 若为RED或者BLACK，触发紧急停止
            emergencyStop();
        } else {
            control_msg_.emergency = false; // 取消紧急状态
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
    ros::Timer flag_timer_; // 标志话题定时器
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
    race_msgs::Flag init_flag_msg_; // 标志话题数据
    // 线程安全锁（避免回调与控制周期并发访问数据）
    std::mutex path_mutex_;
    std::mutex status_mutex_;
    std::mutex flag_mutex_; // 标志话题数据锁
    // 核心参数
    double control_freq_; // 控制频率（Hz）
    double flag_check_freq_; // 标志检查频率（Hz）
    double time_last_flag_check_; // 上次检查标志时间
    double flag_timeout_; // 标志超时时间（s）
    std::string ego_frame_id_; // ego车辆坐标系
    std::string local_path_topic_; // 局部路径话题
    std::string vehicle_state_topic_; // 车辆状态话题
    std::string control_topic_; // 控制指令话题
    std::string flag_topic_; // 标志话题
    // 时间戳（计算dt）
    ros::Time last_control_time_;
    double speed_limit_; // 速度限制（m/s）
    double default_speed_limit_; // 默认速度限制（m/s）
    bool dual_axis_steer_; // 转向模式
    // 横向误差和航向误差阈值
    double lateral_error_threshold_;
    double heading_error_threshold_;
    double default_min_target_speed_; // 在各种绿色旗帜下因航向误差或横向误差而降低到的最小目标速度（km/h）
    // 横向误差和航向误差超出阈值时，目标速度的降低比例
    double lateral_error_decrease_ratio_;
    double heading_error_decrease_ratio_;



};

} // namespace race_tracker

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_manager");
    race_tracker::ControllerManager cm;
    ros::spin();
    return 0;
}