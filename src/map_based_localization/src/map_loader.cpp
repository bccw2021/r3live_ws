/**
 * @file map_loader.cpp
 * @brief 点云地图加载器节点
 * @author Your Name
 * @date 2024
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <std_srvs/Empty.h>

class MapLoader
{
private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::ServiceServer reload_service_;
    ros::Timer map_publish_timer_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;
    std::string map_file_path_;
    std::string map_frame_;
    double voxel_size_;
    bool filter_outliers_;
    double publish_rate_;
    
public:
    MapLoader() : nh_("~")
    {
        // 参数读取
        nh_.param<std::string>("map_file_path", map_file_path_, "");
        nh_.param<std::string>("map_frame", map_frame_, "map");
        nh_.param<double>("voxel_size", voxel_size_, 0.1);
        nh_.param<bool>("filter_outliers", filter_outliers_, true);
        nh_.param<double>("publish_rate", publish_rate_, 0.1); // 10秒发布一次
        
        // 发布者
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);
        
        // 重新加载服务
        reload_service_ = nh_.advertiseService("reload_map", &MapLoader::reloadMapService, this);
        
        // 初始化点云
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // 加载地图
        if(!map_file_path_.empty())
        {
            if(loadMap())
            {
                // 创建定时发布器
                map_publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                                   &MapLoader::publishMapCallback, this);
            }
        }
        else
        {
            ROS_ERROR("未指定地图文件路径！请设置参数 map_file_path");
        }
        
        ROS_INFO("地图加载节点已启动");
        ROS_INFO("地图文件: %s", map_file_path_.c_str());
        ROS_INFO("地图坐标系: %s", map_frame_.c_str());
        ROS_INFO("体素滤波尺寸: %.3f", voxel_size_);
        ROS_INFO("离群点滤波: %s", filter_outliers_ ? "启用" : "禁用");
    }
    
    bool loadMap()
    {
        ROS_INFO("正在加载地图: %s", map_file_path_.c_str());
        
        // 检查文件是否存在
        std::ifstream file(map_file_path_);
        if (!file.good()) {
            ROS_ERROR("地图文件不存在: %s", map_file_path_.c_str());
            return false;
        }
        
        // 加载PCD文件
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(map_file_path_, *global_map_) == -1)
        {
            ROS_ERROR("无法加载地图文件: %s", map_file_path_.c_str());
            return false;
        }
        
        ROS_INFO("原始地图点数: %zu", global_map_->size());
        
        if(global_map_->empty())
        {
            ROS_ERROR("加载的地图为空！");
            return false;
        }
        
        // 预处理地图
        preprocessMap();
        
        // 立即发布一次地图
        publishMap();
        
        ROS_INFO("地图加载完成，处理后点数: %zu", global_map_->size());
        return true;
    }
    
    void preprocessMap()
    {
        // 1. 体素滤波下采样
        if(voxel_size_ > 0)
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
            voxel_filter.setInputCloud(global_map_);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*global_map_);
            ROS_INFO("体素滤波后点数: %zu", global_map_->size());
        }
        
        // 2. 统计滤波去除离群点
        if(filter_outliers_)
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(global_map_);
            sor.setMeanK(50);
            sor.setStddevMulThresh(1.0);
            sor.filter(*global_map_);
            ROS_INFO("离群点滤波后点数: %zu", global_map_->size());
        }
        
        // 3. 可选：高度滤波（只保留一定高度范围的点）
        // pcl::PassThrough<pcl::PointXYZRGB> pass;
        // pass.setInputCloud(global_map_);
        // pass.setFilterFieldName("z");
        // pass.setFilterLimits(-1.0, 3.0);
        // pass.filter(*global_map_);
    }
    
    void publishMap()
    {
        if(global_map_->empty()) 
        {
            ROS_WARN("地图为空，无法发布");
            return;
        }
        
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.stamp = ros::Time::now();
        map_msg.header.frame_id = map_frame_;
        map_pub_.publish(map_msg);
        
        ROS_DEBUG("发布地图，点数: %zu", global_map_->size());
    }
    
    void publishMapCallback(const ros::TimerEvent& event)
    {
        publishMap();
    }
    
    bool reloadMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        ROS_INFO("收到重新加载地图请求");
        return loadMap();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");
    
    try
    {
        MapLoader loader;
        ros::spin();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("地图加载器异常: %s", e.what());
        return -1;
    }
    
    return 0;
}
