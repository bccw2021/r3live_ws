/**
 * @file r3live_map_converter.cpp
 * @brief R3LIVE地图转换工具
 * @author Your Name
 * @date 2024
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

class R3LiveMapConverter
{
public:
    /**
     * @brief 从bag文件中提取点云地图并保存为PCD文件
     * @param bag_file bag文件路径
     * @param topic_name 点云话题名称
     * @param output_file 输出PCD文件路径
     * @return 转换是否成功
     */
    static bool convertBagToPCD(const std::string& bag_file, 
                               const std::string& topic_name,
                               const std::string& output_file)
    {
        ROS_INFO("开始转换bag文件: %s", bag_file.c_str());
        ROS_INFO("点云话题: %s", topic_name.c_str());
        ROS_INFO("输出文件: %s", output_file.c_str());
        
        rosbag::Bag bag;
        try
        {
            bag.open(bag_file, rosbag::bagmode::Read);
        }
        catch(rosbag::BagException& e)
        {
            ROS_ERROR("无法打开bag文件: %s", e.what());
            return false;
        }
        
        std::vector<std::string> topics;
        topics.push_back(topic_name);
        
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        int message_count = 0;
        
        ROS_INFO("开始处理点云消息...");
        
        for(rosbag::MessageInstance const m : view)
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if(cloud_msg != nullptr)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromROSMsg(*cloud_msg, *cloud);
                
                // 累积点云
                *accumulated_cloud += *cloud;
                message_count++;
                
                if(message_count % 100 == 0)
                {
                    ROS_INFO("已处理 %d 条消息，累积点数: %zu", message_count, accumulated_cloud->size());
                }
            }
        }
        
        bag.close();
        
        if(accumulated_cloud->empty())
        {
            ROS_ERROR("未找到有效的点云数据");
            return false;
        }
        
        ROS_INFO("总共处理 %d 条消息，最终点数: %zu", message_count, accumulated_cloud->size());
        
        // 保存PCD文件
        ROS_INFO("正在保存PCD文件...");
        if(pcl::io::savePCDFileBinary(output_file, *accumulated_cloud) == 0)
        {
            ROS_INFO("地图转换完成: %s (点数: %zu)", output_file.c_str(), accumulated_cloud->size());
            return true;
        }
        else
        {
            ROS_ERROR("保存PCD文件失败");
            return false;
        }
    }
    
    /**
     * @brief 从实时话题保存点云地图
     * @param topic_name 点云话题名称
     * @param output_file 输出PCD文件路径
     * @param duration 录制时长（秒）
     * @return 保存是否成功
     */
    static bool saveMapFromTopic(const std::string& topic_name,
                                const std::string& output_file,
                                double duration = 30.0)
    {
        ros::NodeHandle nh;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        ROS_INFO("开始从话题 %s 录制地图，时长: %.1f秒", topic_name.c_str(), duration);
        
        // 创建订阅者
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
            topic_name, 10, 
            [&accumulated_cloud](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromROSMsg(*msg, *cloud);
                *accumulated_cloud += *cloud;
                ROS_INFO_THROTTLE(2, "累积点数: %zu", accumulated_cloud->size());
            });
        
        // 录制指定时长
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(10);
        
        while(ros::ok() && (ros::Time::now() - start_time).toSec() < duration)
        {
            ros::spinOnce();
            rate.sleep();
        }
        
        if(accumulated_cloud->empty())
        {
            ROS_ERROR("未录制到任何点云数据");
            return false;
        }
        
        // 保存PCD文件
        ROS_INFO("正在保存PCD文件...");
        if(pcl::io::savePCDFileBinary(output_file, *accumulated_cloud) == 0)
        {
            ROS_INFO("地图保存完成: %s (点数: %zu)", output_file.c_str(), accumulated_cloud->size());
            return true;
        }
        else
        {
            ROS_ERROR("保存PCD文件失败");
            return false;
        }
    }
};

void printUsage(const char* program_name)
{
    std::cout << "用法:\n";
    std::cout << "  从bag文件转换:\n";
    std::cout << "    " << program_name << " bag <bag_file> <topic_name> <output_pcd>\n";
    std::cout << "  从实时话题保存:\n";
    std::cout << "    " << program_name << " topic <topic_name> <output_pcd> [duration_seconds]\n";
    std::cout << "\n示例:\n";
    std::cout << "  " << program_name << " bag data.bag /r3live/map output_map.pcd\n";
    std::cout << "  " << program_name << " topic /r3live/map indoor_map.pcd 30\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "r3live_map_converter");
    
    if(argc < 4)
    {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string mode = argv[1];
    
    if(mode == "bag")
    {
        if(argc != 5)
        {
            printUsage(argv[0]);
            return 1;
        }
        
        std::string bag_file = argv[2];
        std::string topic_name = argv[3];
        std::string output_file = argv[4];
        
        if(R3LiveMapConverter::convertBagToPCD(bag_file, topic_name, output_file))
        {
            ROS_INFO("转换成功！");
            return 0;
        }
        else
        {
            ROS_ERROR("转换失败！");
            return 1;
        }
    }
    else if(mode == "topic")
    {
        if(argc < 4 || argc > 5)
        {
            printUsage(argv[0]);
            return 1;
        }
        
        std::string topic_name = argv[2];
        std::string output_file = argv[3];
        double duration = (argc == 5) ? std::atof(argv[4]) : 30.0;
        
        if(R3LiveMapConverter::saveMapFromTopic(topic_name, output_file, duration))
        {
            ROS_INFO("保存成功！");
            return 0;
        }
        else
        {
            ROS_ERROR("保存失败！");
            return 1;
        }
    }
    else
    {
        ROS_ERROR("未知模式: %s", mode.c_str());
        printUsage(argv[0]);
        return 1;
    }
}
