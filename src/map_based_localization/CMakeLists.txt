cmake_minimum_required(VERSION 3.0.2)
project(map_based_localization)

## 编译标准
add_compile_options(-std=c++14)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  pcl_ros
  pcl_conversions
  laser_geometry
  std_srvs
)

## 查找PCL库
find_package(PCL REQUIRED)

## catkin特定配置
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    nav_msgs 
    tf 
    pcl_ros 
    pcl_conversions 
    laser_geometry 
    std_srvs
  DEPENDS PCL
)

## 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## 链接目录
link_directories(${PCL_LIBRARY_DIRS})

## 添加PCL定义
add_definitions(${PCL_DEFINITIONS})

## 地图加载器节点
add_executable(map_loader src/map_loader.cpp)
target_link_libraries(map_loader
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## 扫描匹配定位器节点
add_executable(scan_matching_localizer src/scan_matching_localizer.cpp)
target_link_libraries(scan_matching_localizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## 3D点云匹配定位器节点
add_executable(pointcloud_matching_localizer src/pointcloud_matching_localizer.cpp)
target_link_libraries(pointcloud_matching_localizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## R3LIVE地图转换工具
add_executable(r3live_map_converter src/r3live_map_converter.cpp)
target_link_libraries(r3live_map_converter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## 安装
install(TARGETS map_loader scan_matching_localizer pointcloud_matching_localizer r3live_map_converter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
