#include "race_global_static_planner.h"

RaceGlobalStaticPlanner::RaceGlobalStaticPlanner()
    : pnh_("~"),
      origin_lat_(0.0),
      origin_lon_(0.0),
      origin_alt_(0.0),
      is_vehicle_status_received_(false)
{
    // 设置中文编码
    setlocale(LC_ALL, "zh_CN.UTF-8");
    // -------------------------- 1. 读取配置参数 --------------------------
    // CSV文件路径（默认：包内config目录下的corsa.csv）
    std::string pkg_path = ros::package::getPath("race_global_static_planner");
    if (!pnh_.getParam("csv_file_path", csv_file_path_))
    {
        
        csv_file_path_ = pkg_path + "/config/corsa.csv";
        ROS_WARN("Param 'csv_file_path' not set, using default: %s", csv_file_path_.c_str());
    }
    else
    {
        csv_file_path_ = pkg_path + "/" + csv_file_path_;
        ROS_INFO("Param 'csv_file_path' set to: %s", csv_file_path_.c_str());
    }

    // 发布频率参数
    pnh_.param("global_path_publish_rate", global_pub_rate_, 1.0);
    pnh_.param("local_path_publish_rate", local_pub_rate_, 10.0);
    pnh_.param("map_frame", map_frame_, std::string("map"));
    pnh_.param("vehicle_status_topic", vehicle_status_topic_, std::string("/race/vehicle_state"));
    pnh_.param("global_path_topic", global_path_topic_, std::string("/race/global_path"));
    pnh_.param("local_path_topic", local_path_topic_, std::string("/race/local_path"));
    pnh_.param("global_path_vis_topic", global_path_vis_topic_, std::string("/race/global_path_vis"));
    pnh_.param("local_path_vis_topic", local_path_vis_topic_, std::string("/race/local_path_vis"));
    pnh_.param("enable_closed_loop", enable_closed_loop_, true);

    // 可视化开关
    pnh_.param("global_visualization_enable", global_vis_enable_, true);
    pnh_.param("local_visualization_enable", local_vis_enable_, true);

    // 局部路径范围参数
    pnh_.param("local_path_forward_distance", local_forward_dist_, 20.0);
    pnh_.param("local_path_backward_distance", local_backward_dist_, 5.0);

    // 距离计算权重
    pnh_.param("distance_weight_x", weight_x_, 1.0);
    pnh_.param("distance_weight_y", weight_y_, 1.0);
    pnh_.param("distance_weight_z", weight_z_, 0.5);
    pnh_.param("distance_weight_heading", weight_heading_, 0.2);

    // 打印参数配置
    ROS_INFO("===================== Planner Params =====================");
    ROS_INFO("CSV File Path: %s", csv_file_path_.c_str());
    ROS_INFO("Global Pub Rate: %.1f Hz, Local Pub Rate: %.1f Hz", global_pub_rate_, local_pub_rate_);
    ROS_INFO("Global Vis: %s, Local Vis: %s", global_vis_enable_ ? "ON" : "OFF", local_vis_enable_ ? "ON" : "OFF");
    ROS_INFO("Local Path Range: Backward %.1f m, Forward %.1f m", local_backward_dist_, local_forward_dist_);
    ROS_INFO("Distance Weights (X/Y/Z/Heading): %.1f/%.1f/%.1f/%.1f", weight_x_, weight_y_, weight_z_, weight_heading_);
    ROS_INFO("==========================================================");

    // -------------------------- 2. 读取全局路径CSV --------------------------
    if (!readGlobalPathFromCSV())
    {
        ROS_FATAL("Failed to read global path from CSV! Node will shutdown.");
        ros::shutdown();
        return;
    }

    // -------------------------- 3. 预计算路径累积距离 --------------------------
    precomputePathDistances();

    // -------------------------- 4. 创建ROS发布者 --------------------------
    // 全局路径发布者（latch=true：新订阅者可立即获取路径）
    global_path_pub_ = nh_.advertise<race_msgs::Path>(global_path_topic_, 1, true);
    if (global_vis_enable_)
    {
        global_path_vis_pub_ = nh_.advertise<nav_msgs::Path>(global_path_vis_topic_, 1, true);
    }

    // 局部路径发布者（队列大小10，避免消息堆积）
    local_path_pub_ = nh_.advertise<race_msgs::Path>(local_path_topic_, 10);
    if (local_vis_enable_)
    {
        local_path_vis_pub_ = nh_.advertise<nav_msgs::Path>(local_path_vis_topic_, 10);
    }

    // -------------------------- 5. 创建ROS订阅者 --------------------------
    vehicle_status_sub_ = nh_.subscribe<race_msgs::VehicleStatus>(
        vehicle_status_topic_, 10, 
        &RaceGlobalStaticPlanner::vehicleStatusCallback, this);

    // -------------------------- 6. 创建全局路径发布定时器 --------------------------
    if (global_pub_rate_ > 0)
    {
        double period = 1.0 / global_pub_rate_;
        global_pub_timer_ = nh_.createTimer(ros::Duration(period), 
                                           &RaceGlobalStaticPlanner::publishGlobalPath, this);
    }
    else
    {
        ROS_WARN("Global path publish rate <= 0, disable global path auto-publish");
    }

    if (local_pub_rate_ > 0)
    {
        double period = 1.0 / local_pub_rate_;
        local_pub_timer_ = nh_.createTimer(ros::Duration(period), 
                                           &RaceGlobalStaticPlanner::publishLocalPath, this);
    }
    else
    {
        ROS_WARN("Local path publish rate <= 0, disable local path auto-publish");
    }

    ROS_INFO("Race Global Static Planner initialized successfully!");
}

bool RaceGlobalStaticPlanner::readGlobalPathFromCSV()
{
    // 打开CSV文件
    std::ifstream csv_file(csv_file_path_);
    if (!csv_file.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s (Check file path)", csv_file_path_.c_str());
        return false;
    }

    std::string line;
    int line_num = 0;
    int expected_point_count = 0;

    try
    {
        // -------------------------- 读取元数据行（第一行） --------------------------
        if (!std::getline(csv_file, line))
        {
            ROS_ERROR("CSV file is empty!");
            csv_file.close();
            return false;
        }
        line_num++;

        // 分割元数据字段
        std::stringstream meta_ss(line);
        std::vector<std::string> meta_tokens;
        std::string token;
        while (std::getline(meta_ss, token, ','))
        {
            meta_tokens.push_back(token);
        }

        // 验证元数据格式（必须包含6个字段）
        if (meta_tokens.size() != 6)
        {
            throw std::invalid_argument("Metadata line must have 6 fields (got " + 
                                        std::to_string(meta_tokens.size()) + ")");
        }

        // 解析元数据
        int fixed_id = std::stoi(meta_tokens[0]);
        int point_count1 = std::stoi(meta_tokens[1]);
        int point_count2 = std::stoi(meta_tokens[2]);
        origin_lat_ = std::stod(meta_tokens[3]);
        origin_lon_ = std::stod(meta_tokens[4]);
        origin_alt_ = std::stod(meta_tokens[5]);

        // 验证元数据合法性
        if (fixed_id != 10)
        {
            throw std::invalid_argument("Fixed ID must be 10 (got " + std::to_string(fixed_id) + ")");
        }
        if (point_count1 != point_count2)
        {
            throw std::invalid_argument("Point count mismatch (col2: " + std::to_string(point_count1) + 
                                        ", col3: " + std::to_string(point_count2) + ")");
        }
        expected_point_count = point_count1;
        ROS_INFO("CSV Metadata: Point Count=%d, Origin(Lat/Lon/Alt)=%.6f/%.6f/%.6f",
                 expected_point_count, origin_lat_, origin_lon_, origin_alt_);

        // -------------------------- 读取轨迹点行（后续所有行） --------------------------
        while (std::getline(csv_file, line))
        {
            line_num++;
            // 跳过空行
            if (line.empty()) continue;

            // 分割轨迹点字段
            std::stringstream point_ss(line);
            std::vector<std::string> point_tokens;
            while (std::getline(point_ss, token, ','))
            {
                point_tokens.push_back(token);
            }

            // 验证轨迹点格式（必须包含6个字段）
            if (point_tokens.size() != 6)
            {
                ROS_WARN("Skip invalid line %d: must have 6 fields (got %zu)", 
                         line_num, point_tokens.size());
                continue;
            }

            // 解析轨迹点数据
            race_msgs::PathPoint path_point;
            path_point.pose.position.x = std::stod(point_tokens[0]);  // X偏移（东向）
            path_point.pose.position.y = std::stod(point_tokens[1]);  // Y偏移（北向）
            path_point.pose.position.z = std::stod(point_tokens[2]);  // Z偏移（天向）
            double heading = std::stod(point_tokens[3]);              // 参考航向角（ENU东向0，逆时针正）
            path_point.velocity = std::stod(point_tokens[4]);         // 期望车速（m/s）
            path_point.curvature = std::stod(point_tokens[5]);        // 轨迹曲率（1/m）

            // 航向角转换：用户定义（东向0）→ TF定义（北向0，顺时针正）
            // double tf_yaw = M_PI_2 - heading;  // 核心转换公式
            double tf_yaw = heading;  // 核心转换公式
            path_point.pose.orientation = tf::createQuaternionMsgFromYaw(tf_yaw);

            // 添加到全局路径集合
            global_path_points_.push_back(path_point);
        }

        // -------------------------- 验证轨迹点数量 --------------------------
        if (global_path_points_.size() != expected_point_count)
        {
            ROS_WARN("Actual point count (%zu) mismatch with metadata (%d)",
                     global_path_points_.size(), expected_point_count);
        }
        else
        {
            ROS_INFO("Successfully read %zu global path points from CSV", global_path_points_.size());
        }
    }
    catch (const std::invalid_argument& e)
    {
        ROS_ERROR("CSV parse error at line %d: Invalid argument - %s", line_num, e.what());
        csv_file.close();
        return false;
    }
    catch (const std::out_of_range& e)
    {
        ROS_ERROR("CSV parse error at line %d: Value out of range - %s", line_num, e.what());
        csv_file.close();
        return false;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("CSV parse error at line %d: Unknown error - %s", line_num, e.what());
        csv_file.close();
        return false;
    }

    // 关闭文件
    csv_file.close();
    return !global_path_points_.empty();
}

void RaceGlobalStaticPlanner::precomputePathDistances()
{
    if (global_path_points_.empty())
    {
        ROS_WARN("Cannot precompute distances: global path is empty");
        return;
    }

    // 初始化累积距离数组（与路径点数量一致）
    global_path_cumulative_dist_.resize(global_path_points_.size(), 0.0);

    // 计算每个点到起点的累积距离（欧氏距离累加）
    for (size_t i = 1; i < global_path_points_.size(); ++i)
    {
        const auto& prev_point = global_path_points_[i-1];
        const auto& curr_point = global_path_points_[i];

        // 计算相邻点欧氏距离
        double dx = curr_point.pose.position.x - prev_point.pose.position.x;
        double dy = curr_point.pose.position.y - prev_point.pose.position.y;
        double dz = curr_point.pose.position.z - prev_point.pose.position.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        // 累积距离 = 前一点累积距离 + 相邻点距离
        global_path_cumulative_dist_[i] = global_path_cumulative_dist_[i-1] + dist;
    }

    ROS_INFO("Precomputed cumulative distances for %zu path points (total length: %.2f m)",
             global_path_cumulative_dist_.size(), global_path_cumulative_dist_.back());
}

void RaceGlobalStaticPlanner::publishGlobalPath(const ros::TimerEvent& event)
{
    if (global_path_points_.empty())
    {
        ROS_WARN("Skip global path publish: global path is empty");
        return;
    }

    // 构建race_msgs::Path消息
    race_msgs::Path global_path_msg;
    global_path_msg.header.frame_id = map_frame_;  // 固定为ENU坐标系
    global_path_msg.header.stamp = ros::Time::now();
    global_path_msg.points = global_path_points_;

    // 发布全局路径
    global_path_pub_.publish(global_path_msg);
    ROS_DEBUG("Published global path (points: %zu)", global_path_msg.points.size());

    // 发布可视化路径（nav_msgs::Path）
    if (global_vis_enable_)
    {
        nav_msgs::Path vis_path = convertToNavPath(global_path_msg);
        global_path_vis_pub_.publish(vis_path);
    }
}

void RaceGlobalStaticPlanner::publishLocalPath(const ros::TimerEvent& event)
{
    // 生成并发布局部路径
    race_msgs::Path local_path = generateLocalPath(current_vehicle_status_);
    if (local_path.points.empty())
    {
        ROS_WARN("Generated local path is empty (check global path or vehicle status)");
        return;
    }

    // 填充局部路径头信息并发布
    local_path.header.frame_id = map_frame_;
    local_path.header.stamp = ros::Time::now();
    local_path_pub_.publish(local_path);
    ROS_INFO("Published local path (points: %zu, range: %.1f~%.1f m)",
             local_path.points.size(),
             global_path_cumulative_dist_[0],  // 仅为示例，实际需计算局部路径范围
             global_path_cumulative_dist_.back());

    // 发布局部路径可视化
    if (local_vis_enable_)
    {
        nav_msgs::Path local_vis_path = convertToNavPath(local_path);
        local_path_vis_pub_.publish(local_vis_path);
    }
}


void RaceGlobalStaticPlanner::vehicleStatusCallback(const race_msgs::VehicleStatus::ConstPtr& msg)
{
    // 缓存当前车辆状态
    current_vehicle_status_ = *msg;
    is_vehicle_status_received_ = true;


}

race_msgs::Path RaceGlobalStaticPlanner::generateLocalPath(const race_msgs::VehicleStatus& status)
{
    race_msgs::Path local_path;

    // 检查全局路径是否有效
    if (global_path_points_.empty() || global_path_cumulative_dist_.empty())
    {
        ROS_ERROR("Cannot generate local path: global path data is invalid");
        return local_path;
    }

    // -------------------------- 1. 获取当前车辆状态（ENU坐标系） --------------------------
    double car_x = status.pose.position.x;
    double car_y = status.pose.position.y;
    double car_z = status.pose.position.z;
    double car_heading = status.euler.yaw;  // 用户定义：东向0，逆时针正

    // -------------------------- 2. 查找全局路径上的最近点 --------------------------
    double min_cost = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;

    for (size_t i = 0; i < global_path_points_.size(); ++i)
    {
        const auto& path_point = global_path_points_[i];

        // 2.1 计算位置距离成本（马氏距离）
        double dx = std::abs(path_point.pose.position.x - car_x);
        double dy = std::abs(path_point.pose.position.y - car_y);
        double dz = std::abs(path_point.pose.position.z - car_z);

        double pos_cost = weight_x_ * dx*dx + weight_y_ * dy*dy + weight_z_ * dz*dz;

        // 2.2 计算航向角距离成本（周期性最短距离）
        // 路径点航向角：从TF四元数转换回用户定义格式
        double tf_yaw = tf::getYaw(path_point.pose.orientation);
        double path_heading = tf_yaw;  // TF→用户定义
        double heading_dist = AngleUtils::angle_norm(path_heading, car_heading);  // 弧度单位
        double heading_cost = weight_heading_ * heading_dist*heading_dist;

        // 2.3 综合成本（位置+航向）
        double total_cost = pos_cost + heading_cost;

        // 2.4 更新最近点
        if (total_cost < min_cost)
        {
            min_cost = total_cost;
            nearest_idx = i;
        }
    }
    ROS_INFO("当前车辆位置：(%.2f, %.2f, %.2f), 最近路径点位置：(%.2f, %.2f, %.2f)", 
             car_x, car_y, car_z, 
             global_path_points_[nearest_idx].pose.position.x, global_path_points_[nearest_idx].pose.position.y, global_path_points_[nearest_idx].pose.position.z);
    // -------------------------- 3. 确定局部路径的点范围 --------------------------
    double nearest_dist = global_path_cumulative_dist_[nearest_idx];
    double start_dist = nearest_dist - local_backward_dist_;  // 局部路径起始距离
    double end_dist = nearest_dist + local_forward_dist_;    // 局部路径结束距离
    double global_path_len = global_path_cumulative_dist_.back();
    if (enable_closed_loop_ && (start_dist < 0.0 || end_dist > global_path_len))
    {
        if (start_dist < 0.0)
        {
            start_dist = start_dist + global_path_len;
        }
        if (end_dist > global_path_len)
        {
            end_dist = end_dist - global_path_len;
        }
        // 找到局部路径的开始索引
        size_t start_idx = global_path_points_.size() - 1;
        for (size_t i = global_path_cumulative_dist_.size() - 1; i > 0; --i)
        {
            if (global_path_cumulative_dist_[i] <= start_dist)
            {
                start_idx = i;
                break;
            }
        }
        // 找到局部路径的结束索引
        size_t end_idx = 0;
        for (size_t i = 0; i < global_path_cumulative_dist_.size(); ++i)
        {
            if (global_path_cumulative_dist_[i] >= end_dist)
            {
                end_idx = i;
                break;
            }
        }
        // -----------------------------提取局部路径点 --------------------------
        for (size_t i = start_idx; i <= (global_path_points_.size() - 1); ++i)
        {
            local_path.points.push_back(global_path_points_[i]);
        }
        for (size_t i = 0; i <= end_idx; ++i)
        {
            local_path.points.push_back(global_path_points_[i]);
        }
        ROS_DEBUG("Local path generated: nearest_idx=%zu, start_idx=%zu, end_idx=%zu, points=%zu",
        nearest_idx, start_idx, end_idx, local_path.points.size());
    }
    else
    {
        // 3.1 找到局部路径的起始索引（第一个≥start_dist的点）
        size_t start_idx = 0;
        for (size_t i = 0; i < global_path_cumulative_dist_.size(); ++i)
        {
            if (global_path_cumulative_dist_[i] >= start_dist)
            {
                start_idx = i;
                break;
            }
        }

        // 3.2 找到局部路径的结束索引（最后一个≤end_dist的点）
        size_t end_idx = global_path_cumulative_dist_.size() - 1;
        for (size_t i = global_path_cumulative_dist_.size() - 1; i > 0; --i)
        {
            if (global_path_cumulative_dist_[i] <= end_dist)
            {
                end_idx = i;
                break;
            }
        }

        // 验证索引合法性
        if (start_idx > end_idx)
        {
            ROS_WARN("Local path index error: start_idx(%zu) > end_idx(%zu)", start_idx, end_idx);
            start_idx = nearest_idx;
            end_idx = nearest_idx;
        }

        // -------------------------- 4. 提取局部路径点 --------------------------
        for (size_t i = start_idx; i <= end_idx; ++i)
        {
            local_path.points.push_back(global_path_points_[i]);
        }
    ROS_DEBUG("Local path generated: nearest_idx=%zu, start_idx=%zu, end_idx=%zu, points=%zu",
             nearest_idx, start_idx, end_idx, local_path.points.size());
    }


    return local_path;
}

nav_msgs::Path RaceGlobalStaticPlanner::convertToNavPath(const race_msgs::Path& race_path)
{
    nav_msgs::Path nav_path;

    // 复制头信息
    nav_path.header = race_path.header;

    // 转换每个PathPoint到PoseStamped
    for (const auto& race_point : race_path.points)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = race_path.header;
        pose_stamped.pose = race_point.pose;
        nav_path.poses.push_back(pose_stamped);
    }

    return nav_path;
}

// 节点主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "race_global_static_planner");
    RaceGlobalStaticPlanner planner;
    ros::spin();
    return 0;
}