#include "utility.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/octree/octree_search.h>
#include <thread>
#include <mutex>
#include <chrono>

class MapResearch : public ParamServer
{
private:
    // ROS publishers and subscribers
    ros::Publisher pubLocalMap;
    ros::Subscriber subCurrentPose;
    ros::Subscriber subInitialPose;
    
    // Point clouds and octree
    pcl::PointCloud<PointType>::Ptr whole_map;
    pcl::octree::OctreePointCloudSearch<PointType>::Ptr octree_whole_map;
    
    // Current pose tracking
    float current_pose[6];  // [roll, pitch, yaw, x, y, z]
    bool has_pose = false;
    bool map_loaded = false;
    std::mutex pose_mutex;
    
    // Map update settings
    double search_cube_size;
    double search_height_size;
    const double update_interval = 2.0; // 2秒更新一次，TODO
    double last_update_time = 0.0;
    
    // Filters
    pcl::VoxelGrid<PointType> downSizeFilterLocalMap;
    pcl::VoxelGrid<PointType> downSizeFilterWholeMap;
    
    // Threading
    std::thread map_update_thread;
    bool thread_running = false;  // 初始化为false
    bool ros_initialized = false; // 添加ROS初始化标志

public:
    MapResearch() : whole_map(new pcl::PointCloud<PointType>()),
                   octree_whole_map(new pcl::octree::OctreePointCloudSearch<PointType>(1.0))
    {
        ROS_INFO("MapResearch constructor started");
        
        // 首先检查参数是否正确加载
        if (!checkParams()) {
            ROS_ERROR("Failed to load required parameters");
            return;
        }
        
        // Initialize publishers and subscribers
        pubLocalMap = nh.advertise<sensor_msgs::PointCloud2>("liorf_localization/map_research/local_map", 1);
        subCurrentPose = nh.subscribe<nav_msgs::Odometry>("liorf_localization/mapping/odometry", 1, 
                                                         &MapResearch::poseHandler, this);

        subInitialPose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
                                                                               &MapResearch::initialPoseHandler, this);
        
        // Initialize search cube size - 使用默认值如果参数未定义
        search_cube_size = 300.0; // 默认300米
        if (nh.hasParam("liorf_localization/ValidCubeSize")) {
            nh.getParam("liorf_localization/ValidCubeSize", search_cube_size);
        }
        
        search_height_size = 50.0; // 默认300米
        if (nh.hasParam("liorf_localization/ValidCubeHeight")) {
            nh.getParam("liorf_localization/ValidCubeHeight", search_height_size);
        }
        
        // Initialize filters - 使用默认值
        double leaf_size = 0.4; // 默认0.4米
        if (nh.hasParam("liorf_localization/surroundingKeyframeMapLeafSize")) {
            nh.getParam("liorf_localization/surroundingKeyframeMapLeafSize", leaf_size);
        }
        downSizeFilterLocalMap.setLeafSize(leaf_size, leaf_size, leaf_size);

        double leaf_size_whole = 0.8; // 默认0.8米
        if (nh.hasParam("liorf_localization/globalMapVisualizationLeafSize")) {
            nh.getParam("liorf_localization/globalMapVisualizationLeafSize", leaf_size_whole);
        }
        downSizeFilterWholeMap.setLeafSize(leaf_size_whole, leaf_size_whole, leaf_size_whole);

        
        // Initialize pose
        for (int i = 0; i < 6; ++i) {
            current_pose[i] = 0.0;
        }
        
        // 标记ROS已初始化
        ros_initialized = true;
        
        // Load whole map
        loadWholeMap();
        
        // Start map update thread only if map is loaded
        if (map_loaded) {
            thread_running = true;
            map_update_thread = std::thread(&MapResearch::mapUpdateLoop, this);
            ROS_INFO("MapResearch node initialized successfully with map loaded");
        } else {
            ROS_WARN("MapResearch node started but map not loaded");
        }
    }
    
    ~MapResearch()
    {
        thread_running = false;
        if (map_update_thread.joinable()) {
            map_update_thread.join();
        }
    }

private:
    bool checkParams()
    {
        // 检查关键参数是否存在
        std::string sensor_type;
        nh.param<std::string>("liorf_localization/sensor", sensor_type, "velodyne");
        
        if (sensor_type.empty()) {
            ROS_ERROR("Sensor type parameter is empty");
            return false;
        }
        
        // 验证sensor类型
        if (sensor_type != "velodyne" && sensor_type != "ouster" && 
            sensor_type != "livox" && sensor_type != "robosense" && 
            sensor_type != "mulran") {
            ROS_ERROR("Invalid sensor type: %s", sensor_type.c_str());
            return false;
        }
        
        ROS_INFO("Using sensor type: %s", sensor_type.c_str());
        return true;
    }
    
    void loadWholeMap()
    {
        // 获取地图路径参数
        std::string pcd_directory;
        std::string map_name;
        
        nh.param<std::string>("liorf_localization/savePCDDirectory", pcd_directory, "/tmp/");
        nh.param<std::string>("liorf_localization/mapname", map_name, "GlobalMap.pcd");
        
        std::string whole_map_path = std::getenv("HOME") + pcd_directory + map_name;
        
        ROS_INFO("Loading whole map from: %s", whole_map_path.c_str());
        
        if (pcl::io::loadPCDFile<PointType>(whole_map_path, *whole_map) == -1)
        {
            ROS_ERROR("Failed to load whole map from: %s", whole_map_path.c_str());
            
        }
        
        ROS_INFO("Whole map loaded with %zu points", whole_map->size());
        
        if (whole_map->size() < 1000)
        {
            ROS_WARN("Whole map too small: %zu points", whole_map->size());
            return;
        }
        
        // 构建八叉树
        ROS_INFO("Building octree for whole map...");
        try {
            octree_whole_map->setInputCloud(whole_map);
            octree_whole_map->addPointsFromInputCloud();
            map_loaded = true;
            ROS_INFO("Octree built successfully");

            // 地图加载完成后，发布降采样地图给Rviz
            publishWholeMapForRviz();
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to build octree: %s", e.what());
            return;
        }
    }

    void publishWholeMapForRviz()
    {
        if (!map_loaded || whole_map->empty()) {
            ROS_WARN("Cannot publish whole map: map not loaded or empty");
            return;
        }
        
        ROS_INFO("Publishing whole map to Rviz with 1m downsampling...");
        
        // 对全地图进行1米降采样
        pcl::PointCloud<PointType>::Ptr whole_map_ds(new pcl::PointCloud<PointType>());
        try {
            downSizeFilterWholeMap.setInputCloud(whole_map);
            downSizeFilterWholeMap.filter(*whole_map_ds);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to downsample whole map: %s", e.what());
            return;
        }
        
        if (whole_map_ds->empty()) {
            ROS_ERROR("Downsampled whole map is empty");
            return;
        }
        
        // 发布降采样后的全地图
        sensor_msgs::PointCloud2 wholeMapMsg;
        pcl::toROSMsg(*whole_map_ds, wholeMapMsg);
        
        if (ros::Time::isValid()) {
            wholeMapMsg.header.stamp = ros::Time::now();
        } else {
            wholeMapMsg.header.stamp = ros::Time(0);
        }
        wholeMapMsg.header.frame_id = "map";
        
        pubLocalMap.publish(wholeMapMsg);
        
        ROS_INFO("Published whole map to Rviz: %zu points (downsampled from %zu points)", 
                 whole_map_ds->size(), whole_map->size());
    }
    
    void poseHandler(const nav_msgs::Odometry::ConstPtr& poseMsg)
    {
        if (!map_loaded || !ros_initialized) return;
        
        std::lock_guard<std::mutex> lock(pose_mutex);
        
        // Extract position
        current_pose[3] = poseMsg->pose.pose.position.x;
        current_pose[4] = poseMsg->pose.pose.position.y;
        current_pose[5] = poseMsg->pose.pose.position.z;
        
        // Extract orientation
        tf::Quaternion q;
        tf::quaternionMsgToTF(poseMsg->pose.pose.orientation, q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        current_pose[0] = roll;
        current_pose[1] = pitch;
        current_pose[2] = yaw;
        
        has_pose = true;
    }
    
    void initialPoseHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
    {
        if (!map_loaded || !ros_initialized) return;
        
        std::lock_guard<std::mutex> lock(pose_mutex);
        
        // Extract position
        current_pose[3] = poseMsg->pose.pose.position.x;
        current_pose[4] = poseMsg->pose.pose.position.y;
        current_pose[5] = poseMsg->pose.pose.position.z;
        
        // Extract orientation
        tf::Quaternion q;
        tf::quaternionMsgToTF(poseMsg->pose.pose.orientation, q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        current_pose[0] = roll;
        current_pose[1] = pitch;
        current_pose[2] = yaw;
        
        has_pose = true;
        
        // 立即触发一次地图更新
        last_update_time = 0.0;
        
        ROS_INFO("Initial pose received: (%.2f, %.2f, %.2f), triggering immediate map update", 
                 current_pose[3], current_pose[4], current_pose[5]);
    }
    
    void mapUpdateLoop()
    {
        ros::Rate rate(10); // 10Hz检查频率
        
        while (ros::ok() && thread_running)
        {
            if (!ros_initialized) {
                rate.sleep();
                continue;
            }
            
            // 确保ros::Time可用
            if (!ros::Time::isValid()) {
                rate.sleep();
                continue;
            }
            
            double current_time = ros::Time::now().toSec();
            
            // 检查是否需要更新地图
            if (has_pose && map_loaded && 
                (current_time - last_update_time) >= update_interval)
            {
                updateLocalMap();
                last_update_time = current_time;
            }
            
            rate.sleep();
        }
    }       
    
    void updateLocalMap()
    {
        if (!map_loaded || whole_map->empty() || !ros_initialized) {
            return;
        }
        
        float pose_copy[6];
        {
            std::lock_guard<std::mutex> lock(pose_mutex);
            for (int i = 0; i < 6; ++i) {
                pose_copy[i] = current_pose[i];
            }
        }
        
        // 获取当前位置
        PointType current_pos;
        current_pos.x = pose_copy[3];
        current_pos.y = pose_copy[4];
        current_pos.z = pose_copy[5];
        
        // 定义搜索立方体的边界
        double half_size = search_cube_size / 2.0;
        double hight_size = search_height_size;
        Eigen::Vector3f min_pt(current_pos.x - half_size, 
                              current_pos.y - half_size, 
                              current_pos.z - hight_size);
        Eigen::Vector3f max_pt(current_pos.x + half_size, 
                             current_pos.y + half_size, 
                           current_pos.z + hight_size);
    
        
        // 使用八叉树搜索立方体范围内的点
        std::vector<int> point_indices;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        try {
            octree_whole_map->boxSearch(min_pt, max_pt, point_indices);
        } catch (const std::exception& e) {
            ROS_ERROR("Octree search failed: %s", e.what());
            return;
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (point_indices.empty())
        {
            ROS_WARN("No points found in search cube around position (%.2f, %.2f, %.2f)", 
                     current_pos.x, current_pos.y, current_pos.z);
            return;
        }
        
        // 构建新的local map
        pcl::PointCloud<PointType>::Ptr new_local_map(new pcl::PointCloud<PointType>());
        new_local_map->reserve(point_indices.size());
        
        for (int idx : point_indices)
        {
            if (idx >= 0 && idx < static_cast<int>(whole_map->points.size())) {
                new_local_map->push_back(whole_map->points[idx]);
            }
        }
        
        if (new_local_map->empty()) {
            ROS_WARN("New local map is empty after filtering");
            return;
        }
        
        // 下采样
        pcl::PointCloud<PointType>::Ptr local_map_ds(new pcl::PointCloud<PointType>());
        try {
            downSizeFilterLocalMap.setInputCloud(new_local_map);
            downSizeFilterLocalMap.filter(*local_map_ds);
        } catch (const std::exception& e) {
            ROS_ERROR("Downsampling failed: %s", e.what());
            return;
        }
        
        if (local_map_ds->size() > 1000)
        {
            // 发布局部地图
            sensor_msgs::PointCloud2 localMapMsg;
            pcl::toROSMsg(*local_map_ds, localMapMsg);
            // 确保ros::Time可用后再设置时间戳
            if (ros::Time::isValid()) {
                localMapMsg.header.stamp = ros::Time::now();
            } else {
                localMapMsg.header.stamp = ros::Time(0);
            }
            localMapMsg.header.frame_id = "map";
            pubLocalMap.publish(localMapMsg);
            
            ROS_INFO("Published local map with %zu points (search took %ld ms, from %zu raw points)", 
                    local_map_ds->size(), duration.count(), point_indices.size());
        }
        else
        {
            ROS_WARN("Local map too small after downsampling: %zu points", local_map_ds->size());
        }   
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "liorf_localization_mapResearch");

    // 初始化ros::Time
    ros::Time::init();
    
    // 等待ROS参数服务器准备就绪
    ros::Duration(1.0).sleep();
    
    try {
        MapResearch mapResearch;
        
        ROS_INFO("\033[1;32m----> Map Research Started.\033[0m");
        
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("MapResearch node failed: %s", e.what());
        return -1;
    }
    
    return 0;
}