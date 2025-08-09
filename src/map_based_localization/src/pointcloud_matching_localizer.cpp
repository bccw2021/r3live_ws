/**
 * @file pointcloud_matching_localizer.cpp
 * @brief 3D点云匹配定位器节点 - 专为Livox MID-360设计
 * @author Your Name
 * @date 2024
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudMatchingLocalizer
{
private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    
    // 订阅者和发布者
    ros::Subscriber cloud_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher aligned_cloud_pub_;
    ros::Publisher fitness_score_pub_;
    ros::Publisher filtered_cloud_pub_;
    
    // TF相关
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    
    // 定位参数
    std::string map_frame_;
    std::string base_frame_;
    std::string lidar_frame_;
    double max_range_;
    double min_range_;
    double max_height_;
    double min_height_;
    
    // 匹配算法参数
    std::string matching_method_; // "icp" or "ndt"
    double icp_max_distance_;
    int icp_max_iterations_;
    double ndt_resolution_;
    double ndt_step_size_;
    
    // 点云预处理参数
    double voxel_leaf_size_;
    bool enable_outlier_filter_;
    int outlier_mean_k_;
    double outlier_stddev_thresh_;
    
    // 当前位姿
    tf::Transform current_pose_;
    bool pose_initialized_;
    bool map_received_;
    
    // ICP/NDT对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    
    // 滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::PassThrough<pcl::PointXYZ> pass_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;
    
    // 统计信息
    int successful_matches_;
    int total_clouds_;
    ros::Time last_match_time_;
    
public:
    PointCloudMatchingLocalizer() : nh_("~"), pose_initialized_(false), map_received_(false),
                                   successful_matches_(0), total_clouds_(0)
    {
        // 参数读取
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<std::string>("lidar_frame", lidar_frame_, "livox");
        nh_.param<double>("max_range", max_range_, 100.0);
        nh_.param<double>("min_range", min_range_, 0.3);
        nh_.param<double>("max_height", max_height_, 3.0);
        nh_.param<double>("min_height", min_height_, -1.0);
        
        nh_.param<std::string>("matching_method", matching_method_, "ndt");
        nh_.param<double>("icp_max_distance", icp_max_distance_, 2.0);
        nh_.param<int>("icp_max_iterations", icp_max_iterations_, 50);
        nh_.param<double>("ndt_resolution", ndt_resolution_, 2.0);
        nh_.param<double>("ndt_step_size", ndt_step_size_, 0.1);
        
        nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.2);
        nh_.param<bool>("enable_outlier_filter", enable_outlier_filter_, true);
        nh_.param<int>("outlier_mean_k", outlier_mean_k_, 50);
        nh_.param<double>("outlier_stddev_thresh", outlier_stddev_thresh_, 1.0);
        
        // 订阅者
        cloud_sub_ = nh_.subscribe("/cloud_in", 10, &PointCloudMatchingLocalizer::cloudCallback, this);
        map_sub_ = nh_.subscribe("/global_map", 1, &PointCloudMatchingLocalizer::mapCallback, this);
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &PointCloudMatchingLocalizer::initialPoseCallback, this);
        
        // 发布者
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/localization/pose", 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/localization/odom", 10);
        aligned_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 10);
        fitness_score_pub_ = nh_.advertise<std_msgs::Float64>("/localization/fitness_score", 10);
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 10);
        
        // 初始化点云
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 配置ICP
        icp_.setMaxCorrespondenceDistance(icp_max_distance_);
        icp_.setMaximumIterations(icp_max_iterations_);
        icp_.setTransformationEpsilon(1e-6);
        icp_.setEuclideanFitnessEpsilon(1e-6);
        
        // 配置NDT
        ndt_.setResolution(ndt_resolution_);
        ndt_.setStepSize(ndt_step_size_);
        ndt_.setTransformationEpsilon(0.01);
        ndt_.setMaximumIterations(35);
        
        // 配置滤波器
        voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        
        outlier_filter_.setMeanK(outlier_mean_k_);
        outlier_filter_.setStddevMulThresh(outlier_stddev_thresh_);
        
        last_match_time_ = ros::Time::now();
        
        ROS_INFO("3D点云匹配定位节点已启动");
        ROS_INFO("匹配方法: %s", matching_method_.c_str());
        ROS_INFO("地图坐标系: %s", map_frame_.c_str());
        ROS_INFO("机器人坐标系: %s", base_frame_.c_str());
        ROS_INFO("激光坐标系: %s", lidar_frame_.c_str());
        ROS_INFO("点云范围: [%.2f, %.2f]m", min_range_, max_range_);
        ROS_INFO("高度范围: [%.2f, %.2f]m", min_height_, max_height_);
        ROS_INFO("体素滤波尺寸: %.3fm", voxel_leaf_size_);
        
        if(matching_method_ == "icp")
        {
            ROS_INFO("ICP参数 - 最大距离: %.3f, 最大迭代: %d", icp_max_distance_, icp_max_iterations_);
        }
        else if(matching_method_ == "ndt")
        {
            ROS_INFO("NDT参数 - 分辨率: %.3f, 步长: %.3f", ndt_resolution_, ndt_step_size_);
        }
        
        ROS_INFO("等待地图和初始位姿...");
    }
    
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // 转换地图点云
        pcl::fromROSMsg(*msg, *global_map_);
        
        // 预处理地图
        preprocessMap();
        
        // 设置目标点云
        if(matching_method_ == "icp")
        {
            icp_.setInputTarget(global_map_);
        }
        else if(matching_method_ == "ndt")
        {
            ndt_.setInputTarget(global_map_);
        }
        
        map_received_ = true;
        ROS_INFO("接收到3D地图，点数: %zu", global_map_->size());
        ROS_INFO("地图边界: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]", 
                 getMinCoord(*global_map_, 0), getMaxCoord(*global_map_, 0),
                 getMinCoord(*global_map_, 1), getMaxCoord(*global_map_, 1),
                 getMinCoord(*global_map_, 2), getMaxCoord(*global_map_, 2));
    }
    
    void preprocessMap()
    {
        if(global_map_->empty()) return;
        
        // 体素滤波下采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_.setInputCloud(global_map_);
        voxel_filter_.filter(*temp_cloud);
        
        ROS_INFO("地图体素滤波: %zu -> %zu", global_map_->size(), temp_cloud->size());
        global_map_ = temp_cloud;
    }
    
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // 设置初始位姿
        tf::poseMsgToTF(msg->pose.pose, current_pose_);
        pose_initialized_ = true;
        
        ROS_INFO("接收到3D初始位姿: [%.3f, %.3f, %.3f]", 
                 current_pose_.getOrigin().x(),
                 current_pose_.getOrigin().y(),
                 current_pose_.getOrigin().z());
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if(!map_received_ || !pose_initialized_)
        {
            if(!map_received_) ROS_WARN_THROTTLE(5, "等待3D地图数据...");
            if(!pose_initialized_) ROS_WARN_THROTTLE(5, "等待初始位姿...");
            return;
        }
        
        total_clouds_++;
        
        // 转换点云
        pcl::fromROSMsg(*msg, *current_cloud_);
        
        if(current_cloud_->empty())
        {
            ROS_WARN("接收到空点云");
            return;
        }
        
        // 预处理点云
        if(!preprocessPointCloud())
        {
            ROS_WARN("点云预处理失败");
            return;
        }
        
        // 执行3D点云匹配
        if(performPointCloudMatching())
        {
            successful_matches_++;
            // 发布定位结果
            publishLocalizationResult();
            last_match_time_ = ros::Time::now();
        }
        
        // 定期输出统计信息
        if(total_clouds_ % 50 == 0)
        {
            double success_rate = (double)successful_matches_ / total_clouds_ * 100.0;
            double avg_freq = successful_matches_ / (ros::Time::now() - last_match_time_).toSec();
            ROS_INFO("3D定位统计: 成功率 %.1f%% (%d/%d), 平均频率: %.1fHz", 
                     success_rate, successful_matches_, total_clouds_, avg_freq);
        }
    }
    
    bool preprocessPointCloud()
    {
        if(current_cloud_->empty()) return false;
        
        filtered_cloud_->clear();
        
        // 1. 距离滤波
        for(const auto& point : current_cloud_->points)
        {
            double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if(range >= min_range_ && range <= max_range_ && 
               point.z >= min_height_ && point.z <= max_height_ &&
               !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
            {
                filtered_cloud_->push_back(point);
            }
        }
        
        if(filtered_cloud_->empty())
        {
            ROS_WARN("距离滤波后点云为空");
            return false;
        }
        
        // 2. 体素滤波下采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter_.setInputCloud(filtered_cloud_);
        voxel_filter_.filter(*temp_cloud);
        filtered_cloud_ = temp_cloud;
        
        // 3. 离群点滤波
        if(enable_outlier_filter_ && filtered_cloud_->size() > outlier_mean_k_)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            outlier_filter_.setInputCloud(filtered_cloud_);
            outlier_filter_.filter(*outlier_filtered);
            filtered_cloud_ = outlier_filtered;
        }
        
        // 4. 转换到地图坐标系
        transformCloudToMapFrame();
        
        // 发布滤波后的点云
        publishFilteredCloud();
        
        ROS_DEBUG("点云预处理: %zu -> %zu", current_cloud_->size(), filtered_cloud_->size());
        
        return !filtered_cloud_->empty();
    }
    
    void transformCloudToMapFrame()
    {
        try
        {
            // 获取lidar到base_link的变换
            tf::StampedTransform lidar_to_base;
            tf_listener_.lookupTransform(base_frame_, lidar_frame_, ros::Time(0), lidar_to_base);
            
            // 应用当前位姿估计
            tf::Transform map_to_base = current_pose_;
            tf::Transform map_to_lidar = map_to_base * lidar_to_base;
            
            // 变换点云
            Eigen::Matrix4f transform_matrix;
            pcl_ros::transformAsMatrix(map_to_lidar, transform_matrix);
            pcl::transformPointCloud(*filtered_cloud_, *filtered_cloud_, transform_matrix);
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("无法获取TF变换: %s", ex.what());
        }
    }
    
    bool performPointCloudMatching()
    {
        if(filtered_cloud_->empty())
        {
            ROS_WARN("滤波后点云为空，无法进行匹配");
            return false;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        double fitness_score = 0.0;
        bool converged = false;
        
        ros::Time start_time = ros::Time::now();
        
        if(matching_method_ == "icp")
        {
            // 使用ICP匹配
            icp_.setInputSource(filtered_cloud_);
            icp_.align(*aligned_cloud);
            
            converged = icp_.hasConverged();
            if(converged)
            {
                fitness_score = icp_.getFitnessScore();
                updatePoseFromTransformation(icp_.getFinalTransformation());
                ROS_DEBUG("3D ICP收敛，适应度: %f", fitness_score);
            }
            else
            {
                ROS_WARN("3D ICP未收敛");
            }
        }
        else if(matching_method_ == "ndt")
        {
            // 使用NDT匹配
            ndt_.setInputSource(filtered_cloud_);
            ndt_.align(*aligned_cloud);
            
            converged = ndt_.hasConverged();
            if(converged)
            {
                fitness_score = ndt_.getFitnessScore();
                updatePoseFromTransformation(ndt_.getFinalTransformation());
                ROS_DEBUG("3D NDT收敛，适应度: %f", fitness_score);
            }
            else
            {
                ROS_WARN("3D NDT未收敛");
            }
        }
        
        double match_time = (ros::Time::now() - start_time).toSec();
        ROS_DEBUG("3D匹配耗时: %.3fs", match_time);
        
        // 发布适应度分数
        std_msgs::Float64 fitness_msg;
        fitness_msg.data = fitness_score;
        fitness_score_pub_.publish(fitness_msg);
        
        // 发布对齐后的点云
        if(converged)
        {
            publishAlignedCloud(aligned_cloud);
        }
        
        return converged;
    }
    
    void updatePoseFromTransformation(const Eigen::Matrix4f& transformation)
    {
        // 从变换矩阵提取位姿
        tf::Matrix3x3 rotation_matrix(
            transformation(0,0), transformation(0,1), transformation(0,2),
            transformation(1,0), transformation(1,1), transformation(1,2),
            transformation(2,0), transformation(2,1), transformation(2,2)
        );
        
        tf::Vector3 translation(
            transformation(0,3), transformation(1,3), transformation(2,3)
        );
        
        tf::Quaternion quaternion;
        rotation_matrix.getRotation(quaternion);
        
        current_pose_.setOrigin(translation);
        current_pose_.setRotation(quaternion);
    }
    
    void publishFilteredCloud()
    {
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud_, filtered_msg);
        filtered_msg.header.stamp = ros::Time::now();
        filtered_msg.header.frame_id = map_frame_;
        filtered_cloud_pub_.publish(filtered_msg);
    }
    
    void publishAlignedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud)
    {
        sensor_msgs::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned_cloud, aligned_msg);
        aligned_msg.header.stamp = ros::Time::now();
        aligned_msg.header.frame_id = map_frame_;
        aligned_cloud_pub_.publish(aligned_msg);
    }
    
    void publishLocalizationResult()
    {
        ros::Time current_time = ros::Time::now();
        
        // 发布TF变换
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(current_pose_, current_time, map_frame_, base_frame_)
        );
        
        // 发布位姿消息
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = map_frame_;
        tf::poseTFToMsg(current_pose_, pose_msg.pose);
        pose_pub_.publish(pose_msg);
        
        // 发布里程计消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_frame_;
        odom_msg.pose.pose = pose_msg.pose;
        // 这里可以添加速度信息
        odom_pub_.publish(odom_msg);
    }
    
    // 辅助函数：获取点云坐标最小值
    float getMinCoord(const pcl::PointCloud<pcl::PointXYZ>& cloud, int axis)
    {
        if(cloud.empty()) return 0.0f;
        float min_val = std::numeric_limits<float>::max();
        for(const auto& point : cloud.points)
        {
            float val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z;
            if(val < min_val) min_val = val;
        }
        return min_val;
    }
    
    // 辅助函数：获取点云坐标最大值
    float getMaxCoord(const pcl::PointCloud<pcl::PointXYZ>& cloud, int axis)
    {
        if(cloud.empty()) return 0.0f;
        float max_val = std::numeric_limits<float>::min();
        for(const auto& point : cloud.points)
        {
            float val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z;
            if(val > max_val) max_val = val;
        }
        return max_val;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_matching_localizer");
    
    try
    {
        PointCloudMatchingLocalizer localizer;
        ros::spin();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("3D点云匹配定位器异常: %s", e.what());
        return -1;
    }
    
    return 0;
}
