/**
 * @file scan_matching_localizer.cpp
 * @brief 扫描匹配定位器节点
 * @author Your Name
 * @date 2024
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
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
#include <laser_geometry/laser_geometry.h>

class ScanMatchingLocalizer
{
private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    
    // 订阅者和发布者
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher aligned_scan_pub_;
    ros::Publisher fitness_score_pub_;
    
    // TF相关
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan_;
    
    // 定位参数
    std::string map_frame_;
    std::string base_frame_;
    std::string laser_frame_;
    double max_range_;
    double min_range_;
    
    // 匹配算法参数
    std::string matching_method_; // "icp" or "ndt"
    double icp_max_distance_;
    int icp_max_iterations_;
    double ndt_resolution_;
    double ndt_step_size_;
    
    // 当前位姿
    tf::Transform current_pose_;
    bool pose_initialized_;
    bool map_received_;
    
    // ICP/NDT对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    
    // 激光几何转换器
    laser_geometry::LaserProjection laser_projector_;
    
    // 统计信息
    int successful_matches_;
    int total_scans_;
    
public:
    ScanMatchingLocalizer() : nh_("~"), pose_initialized_(false), map_received_(false),
                             successful_matches_(0), total_scans_(0)
    {
        // 参数读取
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<std::string>("laser_frame", laser_frame_, "laser");
        nh_.param<double>("max_range", max_range_, 20.0);
        nh_.param<double>("min_range", min_range_, 0.3);
        
        nh_.param<std::string>("matching_method", matching_method_, "icp");
        nh_.param<double>("icp_max_distance", icp_max_distance_, 1.0);
        nh_.param<int>("icp_max_iterations", icp_max_iterations_, 50);
        nh_.param<double>("ndt_resolution", ndt_resolution_, 1.0);
        nh_.param<double>("ndt_step_size", ndt_step_size_, 0.1);
        
        // 订阅者
        scan_sub_ = nh_.subscribe("/scan", 10, &ScanMatchingLocalizer::scanCallback, this);
        map_sub_ = nh_.subscribe("/global_map", 1, &ScanMatchingLocalizer::mapCallback, this);
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &ScanMatchingLocalizer::initialPoseCallback, this);
        
        // 发布者
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/localization/pose", 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/localization/odom", 10);
        aligned_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_scan", 10);
        fitness_score_pub_ = nh_.advertise<std_msgs::Float64>("/localization/fitness_score", 10);
        
        // 初始化点云
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        current_scan_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        
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
        
        ROS_INFO("扫描匹配定位节点已启动");
        ROS_INFO("匹配方法: %s", matching_method_.c_str());
        ROS_INFO("地图坐标系: %s", map_frame_.c_str());
        ROS_INFO("机器人坐标系: %s", base_frame_.c_str());
        ROS_INFO("激光坐标系: %s", laser_frame_.c_str());
        ROS_INFO("扫描范围: [%.2f, %.2f]", min_range_, max_range_);
        
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
        ROS_INFO("接收到地图，点数: %zu", global_map_->size());
        ROS_INFO("地图边界: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]", 
                 getMinCoord(*global_map_, 0), getMaxCoord(*global_map_, 0),
                 getMinCoord(*global_map_, 1), getMaxCoord(*global_map_, 1),
                 getMinCoord(*global_map_, 2), getMaxCoord(*global_map_, 2));
    }
    
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        // 设置初始位姿
        tf::poseMsgToTF(msg->pose.pose, current_pose_);
        pose_initialized_ = true;
        
        ROS_INFO("接收到初始位姿: [%.3f, %.3f, %.3f]", 
                 current_pose_.getOrigin().x(),
                 current_pose_.getOrigin().y(),
                 tf::getYaw(current_pose_.getRotation()));
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if(!map_received_ || !pose_initialized_)
        {
            if(!map_received_) ROS_WARN_THROTTLE(5, "等待地图数据...");
            if(!pose_initialized_) ROS_WARN_THROTTLE(5, "等待初始位姿...");
            return;
        }
        
        total_scans_++;
        
        // 转换激光扫描到点云
        if(!convertScanToPointCloud(msg))
        {
            ROS_WARN("激光扫描转换失败");
            return;
        }
        
        // 执行扫描匹配
        if(performScanMatching())
        {
            successful_matches_++;
            // 发布定位结果
            publishLocalizationResult();
        }
        
        // 定期输出统计信息
        if(total_scans_ % 100 == 0)
        {
            double success_rate = (double)successful_matches_ / total_scans_ * 100.0;
            ROS_INFO("定位统计: 成功率 %.1f%% (%d/%d)", success_rate, successful_matches_, total_scans_);
        }
    }
    
    bool convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        try
        {
            // 使用laser_geometry转换激光扫描到点云
            sensor_msgs::PointCloud2 cloud_msg;
            laser_projector_.projectLaser(*scan_msg, cloud_msg);
            
            // 转换到PCL点云
            pcl::fromROSMsg(cloud_msg, *current_scan_);
            
            // 过滤点云
            filterScan();
            
            if(current_scan_->empty())
            {
                ROS_WARN("过滤后的扫描点云为空");
                return false;
            }
            
            // 转换到地图坐标系
            transformScanToMapFrame();
            
            return true;
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("激光扫描转换异常: %s", e.what());
            return false;
        }
    }
    
    void filterScan()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZ>);
        
        for(const auto& point : current_scan_->points)
        {
            double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            
            // 过滤无效点
            if(range >= min_range_ && range <= max_range_ && 
               !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
            {
                filtered_scan->push_back(point);
            }
        }
        
        current_scan_ = filtered_scan;
        ROS_DEBUG("过滤后扫描点数: %zu", current_scan_->size());
    }
    
    void transformScanToMapFrame()
    {
        try
        {
            // 获取laser到base_link的变换
            tf::StampedTransform laser_to_base;
            tf_listener_.lookupTransform(base_frame_, laser_frame_, ros::Time(0), laser_to_base);
            
            // 应用当前位姿估计
            tf::Transform map_to_base = current_pose_;
            tf::Transform map_to_laser = map_to_base * laser_to_base;
            
            // 变换点云
            Eigen::Matrix4f transform_matrix;
            pcl_ros::transformAsMatrix(map_to_laser, transform_matrix);
            pcl::transformPointCloud(*current_scan_, *current_scan_, transform_matrix);
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("无法获取TF变换: %s", ex.what());
        }
    }
    
    bool performScanMatching()
    {
        if(current_scan_->empty())
        {
            ROS_WARN("当前扫描为空，无法进行匹配");
            return false;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan(new pcl::PointCloud<pcl::PointXYZ>);
        double fitness_score = 0.0;
        bool converged = false;
        
        if(matching_method_ == "icp")
        {
            // 使用ICP匹配
            icp_.setInputSource(current_scan_);
            icp_.align(*aligned_scan);
            
            converged = icp_.hasConverged();
            if(converged)
            {
                fitness_score = icp_.getFitnessScore();
                updatePoseFromTransformation(icp_.getFinalTransformation());
                ROS_DEBUG("ICP收敛，适应度: %f", fitness_score);
            }
            else
            {
                ROS_WARN("ICP未收敛");
            }
        }
        else if(matching_method_ == "ndt")
        {
            // 使用NDT匹配
            ndt_.setInputSource(current_scan_);
            ndt_.align(*aligned_scan);
            
            converged = ndt_.hasConverged();
            if(converged)
            {
                fitness_score = ndt_.getFitnessScore();
                updatePoseFromTransformation(ndt_.getFinalTransformation());
                ROS_DEBUG("NDT收敛，适应度: %f", fitness_score);
            }
            else
            {
                ROS_WARN("NDT未收敛");
            }
        }
        
        // 发布适应度分数
        std_msgs::Float64 fitness_msg;
        fitness_msg.data = fitness_score;
        fitness_score_pub_.publish(fitness_msg);
        
        // 发布对齐后的扫描
        if(converged)
        {
            publishAlignedScan(aligned_scan);
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
    
    void publishAlignedScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_scan)
    {
        sensor_msgs::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned_scan, aligned_msg);
        aligned_msg.header.stamp = ros::Time::now();
        aligned_msg.header.frame_id = map_frame_;
        aligned_scan_pub_.publish(aligned_msg);
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
    ros::init(argc, argv, "scan_matching_localizer");
    
    try
    {
        ScanMatchingLocalizer localizer;
        ros::spin();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("扫描匹配定位器异常: %s", e.what());
        return -1;
    }
    
    return 0;
}
