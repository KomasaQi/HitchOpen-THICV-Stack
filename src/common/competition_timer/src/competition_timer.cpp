#include "competition_timer.h"

// -------------------------- 构造函数实现 --------------------------
CompetitionTimer::CompetitionTimer() 
    : current_flag(race_msgs::Flag::RED)
    , previous_flag(race_msgs::Flag::RED)
    , start_time(0)
    , lap_start_time(0)
    , current_lap(0)
    , total_laps(0)
    , is_closed_loop(false)
    , publish_rate(10.0)
    , distance_threshold_to_leave_start(0.0)
    , race_state(BEFORE_START)
    , private_nh("~")
{
    // 设置中文编码（避免日志乱码）
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    // 初始化旗帜映射表
    initFlagMappings();
    
    // 等待参数加载（最多等待2秒，每100ms检查一次）
    int wait_count = 0;
    while (!ros::param::has("/competition_timer/flag_topic_name") && wait_count < 20) {
        ROS_INFO("等待参数加载...");
        ros::Duration(0.1).sleep();
        wait_count++;
    }

    // 加载参数
    loadParameters();
    
    // 创建发布者和订阅者
    flag_pub = nh.advertise<race_msgs::Flag>(flag_topic_name, 10);
    info_pub = nh.advertise<std_msgs::String>(info_topic_name, 10);
    vehicle_status_sub = nh.subscribe(vehicle_status_topic_name, 10, 
        &CompetitionTimer::vehicleStatusCallback, this);
    
    // 创建定时器（按publish_rate触发回调）
    timer = nh.createTimer(ros::Duration(1.0 / publish_rate), 
        &CompetitionTimer::timerCallback, this);
    
    // 初始化离开起点的距离阈值（起点检测距离+1m，避免误判）
    distance_threshold_to_leave_start = start_detection_distance + 1.0;

    race_state = 0; // 0: 未开始

    // 打印初始化日志
    ROS_INFO_STREAM("Competition Timer Node 初始化完成，当前旗帜状态为: " << getFlagName(current_flag) );
    ROS_INFO_STREAM("发布比赛旗帜到话题: " << flag_topic_name);
    ROS_INFO_STREAM("发布比赛信息到话题: " << info_topic_name);
    ROS_INFO_STREAM("订阅车辆状态话题名称: " << vehicle_status_topic_name);
    publishInfo("Competition Timer Node 启动完成，当前旗帜状态为: " + getFlagName(current_flag) );
}

// -------------------------- 旗帜映射初始化 --------------------------
void CompetitionTimer::initFlagMappings() {
    // 名称→数值映射（对应race_msgs/Flag.msg的定义）
    flag_name_to_value["RED"] = race_msgs::Flag::RED;
    flag_name_to_value["GREEN"] = race_msgs::Flag::GREEN;
    flag_name_to_value["BLACK"] = race_msgs::Flag::BLACK;
    flag_name_to_value["G5"] = race_msgs::Flag::G5;
    flag_name_to_value["G10"] = race_msgs::Flag::G10;
    flag_name_to_value["G15"] = race_msgs::Flag::G15;
    flag_name_to_value["G20"] = race_msgs::Flag::G20;
    flag_name_to_value["G40"] = race_msgs::Flag::G40;
    flag_name_to_value["G60"] = race_msgs::Flag::G60;
    flag_name_to_value["G80"] = race_msgs::Flag::G80;

    // 数值→名称反向映射（用于日志打印）
    for (auto& pair : flag_name_to_value) {
        flag_value_to_name[pair.second] = pair.first;
    }
}

// -------------------------- 参数加载实现 --------------------------
void CompetitionTimer::loadParameters() {
    // 1. 加载话题名称（私有参数空间）
    private_nh.param<std::string>("flag_topic_name", flag_topic_name, "/competition_flag");
    private_nh.param<std::string>("info_topic_name", info_topic_name, "/competition_info");
    private_nh.param<std::string>("vehicle_status_topic_name", vehicle_status_topic_name, "/vehicle_status");

    // 2. 加载比赛配置参数
    private_nh.param("publish_rate", publish_rate, 10.0);          // 发布频率（Hz）
    private_nh.param("total_laps", total_laps, 3);                // 总圈数
    private_nh.param("is_closed_loop", is_closed_loop, true);     // 是否闭环赛道
    private_nh.param("start_detection_distance", start_detection_distance, 2.0); // 起点检测阈值（m）
    private_nh.param("end_detection_distance", end_detection_distance, 2.0);     // 终点检测阈值（m）

    // 3. 加载起点坐标（参数为vector<double>，默认(0,0,0)）
    std::vector<double> start_point(3, 0.0);
    private_nh.param("start_point", start_point, start_point);
    start_point_.x = start_point[0];
    start_point_.y = start_point[1];
    start_point_.z = start_point[2];

    // 4. 加载终点坐标（默认(0,0,0)）
    std::vector<double> end_point(3, 0.0);
    private_nh.param("end_point", end_point, end_point);
    end_point_.x = end_point[0];
    end_point_.y = end_point[1];
    end_point_.z = end_point[2];

    // 5. 加载初始旗帜（强制默认为RED，忽略参数服务器旧值）
    // 关键修改：无论参数服务器中是否有值，启动时都强制设为RED
    std::string initial_flag_name = "RED"; 
    // 如需允许用户启动时通过命令行参数覆盖，可取消下面一行注释
    private_nh.param("initial_flag", initial_flag_name, std::string("RED"));
    
    // 清除参数服务器中的旧值，确保下次启动时使用默认RED
    private_nh.setParam("flag", "RED");
    
    if (flag_name_to_value.find(initial_flag_name) != flag_name_to_value.end()) {
        current_flag = flag_name_to_value[initial_flag_name];
    } else {
        ROS_WARN_STREAM("无效初始旗帜名称: " << initial_flag_name << ", 已默认使用RED");
        current_flag = race_msgs::Flag::RED;
    }
    
    // 打印关键参数配置，方便调试
    ROS_INFO_STREAM("比赛配置 - 总圈数: " << total_laps << ", 赛道类型: " << (is_closed_loop ? "闭环" : "开环"));
    ROS_INFO_STREAM("起点坐标: (" << start_point_.x << ", " << start_point_.y << ", " << start_point_.z << "), 检测阈值: " << start_detection_distance << "m");
    ROS_INFO_STREAM("终点坐标: (" << end_point_.x << ", " << end_point_.y << ", " << end_point_.z << "), 检测阈值: " << end_detection_distance << "m");
}

// -------------------------- 车辆状态回调实现 --------------------------
void CompetitionTimer::vehicleStatusCallback(const race_msgs::VehicleStatus::ConstPtr& msg) {
    if (race_state == FINISHED) return; // 比赛结束后不再处理

    // 更新当前车辆位置
    current_position_ = msg->pose.position;
    
    // 调试日志：打印当前位置信息
    ROS_DEBUG_STREAM("车辆位置: (" << current_position_.x << ", " << current_position_.y << ", " << current_position_.z << ")");

    // 1. 比赛未开始且旗帜为绿色：检测车辆是否到达起点
    if (race_state == BEFORE_START && isGreenFlag(current_flag)) {
        double distance_to_start = calculateDistance(current_position_, start_point_);
        
        if (distance_to_start <= start_detection_distance) {
            race_state = IN_START;  // 标记正在起点区域
            current_lap = 1;          // 初始圈数为1
            start_time = ros::Time::now();  // 记录比赛开始时间
            lap_start_time = start_time;    // 记录当前圈开始时间
            // 发布开始信息
            std::stringstream ss;
            ss << "比赛开始，旗帜状态: " << getFlagName(current_flag) 
            << "，需要完成的总圈数: " << total_laps;
            publishInfo(ss.str());
            ROS_INFO_STREAM(ss.str());

        }
    }
    // 2. 在起点范围内，检查是否出起点
    if (race_state == IN_START) {
        // 圈数达到总圈数→结束比赛
        if (current_lap > total_laps) {
            finishRace();
            race_state = FINISHED;
        }
        double distance_to_start = calculateDistance(current_position_, start_point_);
        if (distance_to_start > start_detection_distance) {
            // 车辆已离开起点区域
            race_state = IN_LAP;  // 切换到圈中状态
            ROS_INFO_STREAM("车辆已离开起点区域，当前开始进行第" << current_lap << "圈");

        }
    }
    // 3. 比赛已开始且未结束：检查圈数/终点
    if (race_state == IN_LAP) {
        if (is_closed_loop) { 
            // 闭环赛道：检测回到起点（需先离开起点）
            double distance_to_start = calculateDistance(current_position_, start_point_);

                // 回到起点→完成一圈
                if (distance_to_start <= start_detection_distance) {
                    completeLap();
                    race_state = IN_START;
                    

                }
        } else { 
            // 开环赛道：直接检测终点
            double distance_to_end = calculateDistance(current_position_, end_point_);
            ROS_DEBUG_STREAM("开环赛道 - 距终点: " << distance_to_end << "m (阈值: " << end_detection_distance << "m)");
            
            if (distance_to_end <= end_detection_distance) {
                
                finishRace();
                race_state = FINISHED;
            }
        }
    }

}



// -------------------------- 定时器回调实现 --------------------------
void CompetitionTimer::timerCallback(const ros::TimerEvent& event) {
    // 检查参数服务器中旗帜是否更新
    checkFlagParamUpdate();

    // 发布当前旗帜状态
    race_msgs::Flag flag_msg;
    flag_msg.flag = current_flag;
    flag_pub.publish(flag_msg);

    // 旗帜状态变化时发布日志
    if (current_flag != previous_flag) {
        std::string flag_name = getFlagName(current_flag);
        // 针对不同的旗帜给出不同的提示
        if (flag_name == "RED") {
            publishInfo("比赛旗帜已更新为: " + flag_name + ", 车辆需立即紧急停止");
            ROS_WARN_STREAM("比赛旗帜已更新为: " << flag_name << ", 车辆需立即紧急停止");
        } else if (flag_name == "GREEN") {
            publishInfo("比赛旗帜已更新为: " + flag_name );
            ROS_INFO_STREAM("比赛旗帜已更新为: " << flag_name);
        } else if (flag_name == "BLACK") {
            publishInfo("比赛旗帜已更新为: " + flag_name + ", 比赛完成");
            ROS_INFO_STREAM("比赛旗帜已更新为: " << flag_name << ", 比赛完成");
        } else if (current_flag > 5u) {
            publishInfo("比赛旗帜已更新为: " + flag_name + ", 当前限速: " + std::to_string(static_cast<int>(current_flag)) + " km/h"); // 显示uint8的限速数值
            ROS_INFO_STREAM("比赛旗帜已更新为: " << flag_name << ", 当前限速: " << static_cast<int>(current_flag) << " km/h");
        } else {
            publishInfo("比赛旗帜已更新为: " + flag_name);
            ROS_INFO_STREAM("比赛旗帜已更新为: " << flag_name);
        } 
        previous_flag = current_flag;
    }
}

// -------------------------- 检查旗帜参数更新 --------------------------
void CompetitionTimer::checkFlagParamUpdate() {
    std::string new_flag_name;
    // 从私有参数空间获取最新旗帜名称
    if (private_nh.getParam("flag", new_flag_name)) {
        // 验证旗帜名称有效性
        if (flag_name_to_value.find(new_flag_name) != flag_name_to_value.end()) {
            uint8_t new_flag = flag_name_to_value[new_flag_name];
            // 仅当旗帜变化时处理
            if (new_flag != current_flag) {
                // 绿色旗帜切换逻辑：未开始比赛时提示等待发车
                if (!isGreenFlag(current_flag) && isGreenFlag(new_flag) && race_state == BEFORE_START) {
                    publishInfo("准备开始, 正在等待车辆到达发车点");
                    // 显示具体发车点位置和当前位置
                    ROS_INFO_STREAM("检测到手动旗帜切换为: " << new_flag_name << ", 等待车辆到达发车点：【"
                         << std::to_string(start_point_.x) << ", " << std::to_string(start_point_.y) << ", " << std::to_string(start_point_.z) << "】"
                        << ", 当前车辆位置：【"
                        << std::to_string(current_position_.x) << ", " << std::to_string(current_position_.y) << ", " << std::to_string(current_position_.z) << "】," 
                        << "距离发车点: " << calculateDistance(current_position_, start_point_) << " m"); 
                }
                current_flag = new_flag;
                ROS_INFO_STREAM("检测到手动旗帜更新为: " << new_flag_name);
            }
        } else if (!new_flag_name.empty()) {
            ROS_WARN_STREAM("无效旗帜名称: " << new_flag_name);
        }
    }
}




// -------------------------- 完成一圈实现 --------------------------
void CompetitionTimer::completeLap() {
    ros::Time lap_end_time = ros::Time::now();
    ros::Duration lap_duration = lap_end_time - lap_start_time; // 圈时
    ros::Duration total_duration = lap_end_time - start_time;   // 总时间

    // 格式化圈完成信息
    std::stringstream ss;
    ss << "第 " << current_lap << " 圈完成. "
       << "圈时: " << formatDuration(lap_duration) << ". "
       << "总时间: " << formatDuration(total_duration) << ".";

    publishInfo(ss.str());
    ROS_INFO_STREAM(ss.str());

    // 更新圈数和下一圈开始时间
    current_lap++;
    lap_start_time = lap_end_time;
}

// -------------------------- 结束比赛实现 --------------------------
void CompetitionTimer::finishRace() {

    current_flag = race_msgs::Flag::BLACK; // 切换为黑旗
    private_nh.setParam("flag", "BLACK");
    // 计算总比赛时间
    ros::Time finish_time = ros::Time::now();
    ros::Duration total_duration = finish_time - start_time;

    // 发布结束信息
    std::stringstream ss;
    ss << "比赛结束! 总时间: " << formatDuration(total_duration) << ". "
       << "完成总圈数: " << (current_lap - 1) << "."; // current_lap已自增，需减1

    publishInfo(ss.str());
    ROS_INFO_STREAM(ss.str());
    publishInfo("黑旗已激活，比赛结束.");
}

// -------------------------- 计算距离实现 --------------------------
double CompetitionTimer::calculateDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// -------------------------- 检查旗帜有效性 --------------------------
bool CompetitionTimer::isValidFlag(uint8_t flag) {
    return flag_value_to_name.find(flag) != flag_value_to_name.end();
}

// -------------------------- 检查绿色系旗帜 --------------------------
bool CompetitionTimer::isGreenFlag(uint8_t flag) {
    std::string name = getFlagName(flag);
    return name == "GREEN" || 
           name == "G5" || 
           name == "G10" || 
           name == "G15" || 
           name == "G20" || 
           name == "G40" || 
           name == "G60" || 
           name == "G80";
}

// -------------------------- 获取旗帜名称 --------------------------
std::string CompetitionTimer::getFlagName(uint8_t flag) {
    if (flag_value_to_name.find(flag) != flag_value_to_name.end()) {
        return flag_value_to_name[flag];
    }
    return "UNKNOWN"; // 无效旗帜返回"未知"
}

// -------------------------- 获取旗帜数值 --------------------------
uint8_t CompetitionTimer::getFlagValue(const std::string& name) {
    if (flag_name_to_value.find(name) != flag_name_to_value.end()) {
        return flag_name_to_value[name];
    }
    return race_msgs::Flag::RED; // 无效名称默认返回RED
}

// -------------------------- 格式化时间实现 --------------------------
std::string CompetitionTimer::formatDuration(const ros::Duration& duration) {
    int hours = duration.sec / 3600;
    int minutes = (duration.sec % 3600) / 60;
    int seconds = duration.sec % 60;
    int milliseconds = duration.nsec / 1000000; // 纳秒→毫秒

    std::stringstream ss;
    if (hours > 0) ss << hours << "h ";
    if (minutes > 0 || hours > 0) ss << minutes << "m ";
    ss << seconds << "." << std::setw(3) << std::setfill('0') << milliseconds << "s";
    return ss.str();
}

// -------------------------- 发布信息实现 --------------------------
void CompetitionTimer::publishInfo(const std::string& message) {
    std_msgs::String info_msg;
    info_msg.data = message;
    info_pub.publish(info_msg);
}

// -------------------------- 节点运行实现 --------------------------
void CompetitionTimer::run() {
    ros::spin(); // 阻塞等待回调（处理订阅和定时器事件）
}

// -------------------------- main函数（节点入口） --------------------------
int main(int argc, char **argv) {
    // 初始化ROS节点（节点名：competition_timer）
    ros::init(argc, argv, "competition_timer");
    
    try {
        CompetitionTimer timer; // 创建定时器对象（自动执行构造函数初始化）
        timer.run();            // 启动节点运行
    } catch (const std::exception& e) {
        ROS_FATAL("Competition Timer Node 启动失败: %s", e.what());
        return 1;
    }
    
    return 0;
}
