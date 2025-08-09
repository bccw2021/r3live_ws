# 基于点云地图的扫描匹配定位系统

## 概述
这是一个基于ROS1的点云地图加载与实时扫描匹配定位系统，支持已录制的点云地图与当前/scan数据的高精度匹配定位。

## 功能特性
- **地图加载器**：支持PCD格式点云地图加载，自动体素滤波和离群点去除
- **3D点云定位**：直接使用Livox MID-360的3D点云数据进行高精度定位
- **2D激光扫描定位**：可选的2D激光扫描匹配定位模式
- **双重匹配算法**：支持ICP和NDT两种匹配算法
- **实时定位**：发布TF变换、位姿和里程计信息
- **智能滤波**：多层点云预处理，包括距离滤波、体素滤波和离群点滤波
- **地图转换工具**：从R3LIVE bag文件或实时话题提取点云地图

## 系统要求
- ROS Noetic
- PCL 1.10+
- Eigen3
- Livox ROS驱动（livox_ros_driver）
- pointcloud_to_laserscan包（用于点云转2D激光扫描）

## 编译安装

### 1. 安装依赖
```bash
# 安装pointcloud_to_laserscan包
sudo apt-get install ros-noetic-pointcloud-to-laserscan

# 确保livox_ros_driver已安装（应该已经在你的工作空间中）
ls ~/r3live_ws/src/livox_ros_driver
```

### 2. 编译
```bash
cd ~/r3live_ws
catkin_make --only-pkg-with-deps map_based_localization
source devel/setup.bash
```

### 3. 检查编译结果
```bash
rospack find map_based_localization
ls ~/r3live_ws/devel/lib/map_based_localization/
```

## Livox MID-360配置

### 1. 获取雷达序列号
```bash
# 连接Livox MID-360后，运行以下命令查看序列号
roslaunch livox_ros_driver livox_lidar_msg.launch
# 在终端输出中找到类似 "3GGDJ6S00100161" 的序列号
```

### 2. 修改Launch文件中的序列号
编辑 `launch/map_based_localization.launch` 文件，将以下行中的序列号替换为你的雷达序列号：
```xml
<arg name="bd_list" value="3GGDJ6S00100161"/>  <!-- 替换为你的雷达序列号 -->
```

### 3. 调整TF变换
根据Livox MID-360在你机器人上的实际安装位置，修改Launch文件中的TF变换参数：
```xml
<!-- 调整x,y,z,roll,pitch,yaw参数 -->
<node pkg="tf" type="static_transform_publisher" name="base_to_livox_tf" 
      args="0.0 0.0 0.3 0 0 0 base_link livox 100" />
```

### 4. Livox点云话题
- 原始点云话题：`/livox/lidar` (sensor_msgs/PointCloud2)
- 转换后的2D扫描话题：`/scan` (sensor_msgs/LaserScan)
- IMU话题：`/livox/imu` (sensor_msgs/Imu)

## 使用方法

### 1. 准备点云地图

#### 方法1：从R3LIVE bag文件提取
```bash
# 从bag文件提取地图
rosrun map_based_localization r3live_map_converter bag data.bag /r3live/map indoor_map.pcd
```

#### 方法2：从实时话题保存
```bash
# 启动R3LIVE系统后，从实时话题保存地图（录制30秒）
rosrun map_based_localization r3live_map_converter topic /r3live/map indoor_map.pcd 30
```

#### 方法3：直接使用R3LIVE输出
```bash
# 复制R3LIVE保存的地图文件
mkdir -p ~/indoor_maps
cp ~/r3live_output/map.pcd ~/indoor_maps/indoor_map.pcd
```

### 2. 启动定位系统

#### 3D点云定位启动（推荐）
```bash
# 启动3D点云定位系统（默认NDT算法）
roslaunch map_based_localization map_based_localization.launch \
    map_file:=~/indoor_maps/indoor_map.pcd \
    laser_frame:=livox
```

#### 高精度3D定位模式
```bash
# 使用ICP算法进行高精度3D定位（计算量较大）
roslaunch map_based_localization map_based_localization.launch \
    map_file:=~/indoor_maps/indoor_map.pcd \
    matching_method:=icp \
    laser_frame:=livox
```

#### 实时3D定位模式
```bash
# 使用NDT算法进行实时3D定位（推荐日常使用）
roslaunch map_based_localization map_based_localization.launch \
    map_file:=~/indoor_maps/indoor_map.pcd \
    matching_method:=ndt \
    laser_frame:=livox
```

### 3. 设置初始位姿
1. 在RViz中使用"2D Pose Estimate"工具
2. 点击地图上机器人的初始位置和朝向
3. 系统将开始实时定位

### 4. 监控定位效果
```bash
# 查看定位位姿
rostopic echo /localization/pose

# 查看里程计信息
rostopic echo /localization/odom

# 监控匹配质量
rostopic echo /localization/fitness_score

# 查看TF树
rosrun tf view_frames
evince frames.pdf
```

## 参数配置

### 地图加载器参数
- `map_file_path`: 地图文件路径
- `map_frame`: 地图坐标系名称（默认：map）
- `voxel_size`: 体素滤波尺寸（默认：0.1）
- `filter_outliers`: 是否启用离群点滤波（默认：true）
- `publish_rate`: 地图发布频率（默认：0.1 Hz）

### 扫描匹配定位器参数
- `matching_method`: 匹配算法（icp/ndt，默认：icp）
- `max_range`: 激光最大距离（默认：20.0）
- `min_range`: 激光最小距离（默认：0.3）

#### ICP参数
- `icp_max_distance`: 对应点最大距离（默认：1.0）
- `icp_max_iterations`: 最大迭代次数（默认：50）

#### NDT参数
- `ndt_resolution`: 网格分辨率（默认：1.0）
- `ndt_step_size`: 步长（默认：0.1）

## 话题接口

### 3D点云定位模式

#### 订阅话题
- `/cloud_registered` (sensor_msgs/PointCloud2): **R3LIVE处理后的高质量点云（推荐）**
- `/livox/lidar` (sensor_msgs/PointCloud2): Livox MID-360原始点云数据（可选）
- `/laser_cloud_flat` (sensor_msgs/PointCloud2): LOAM平面特征点云（可选）
- `/global_map` (sensor_msgs/PointCloud2): 全局点云地图
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): 初始位姿

#### 发布话题
- `/localization/pose` (geometry_msgs/PoseStamped): 3D定位位姿
- `/localization/odom` (nav_msgs/Odometry): 里程计信息
- `/aligned_cloud` (sensor_msgs/PointCloud2): 对齐后的3D点云
- `/filtered_cloud` (sensor_msgs/PointCloud2): 预处理后的点云
- `/localization/fitness_score` (std_msgs/Float64): 匹配适应度分数

### 🎯 **点云数据源选择建议**

#### **1. `/cloud_registered` - R3LIVE处理后点云（强烈推荐）**
**优势：**
- ✅ **运动补偿**：基于IMU数据进行精确的运动补偿
- ✅ **状态估计优化**：结合LiDAR-IMU紧耦合状态估计
- ✅ **质量过滤**：移除低质量点和噪声点
- ✅ **坐标系统一**：已转换到世界坐标系
- ✅ **时间同步**：精确的时间戳对齐

**适用场景：**
- 高精度室内定位
- 动态环境定位
- 需要最佳定位精度的应用

#### **2. `/laser_cloud_flat` - LOAM平面特征点云**
**优势：**
- ✅ **特征提取**：专门的平面特征点
- ✅ **计算效率**：点云数量较少，计算快速
- ✅ **几何优化**：适合平面丰富的室内环境

**适用场景：**
- 计算资源受限的场景
- 平面特征丰富的室内环境
- 实时性要求极高的应用

#### **3. `/livox/lidar` - 原始Livox点云**
**优势：**
- ✅ **完整信息**：保留所有原始点云数据
- ✅ **低延迟**：直接来源，延迟最小

**劣势：**
- ❌ **无运动补偿**：可能存在运动畸变
- ❌ **噪声较多**：包含低质量点和噪声
- ❌ **计算量大**：需要更多预处理

**适用场景：**
- 静态或低速运动场景
- 需要完整点云信息的特殊应用

### 2D激光扫描定位模式（可选）

#### 订阅话题
- `/scan` (sensor_msgs/LaserScan): 转换后的2D激光扫描数据
- `/global_map` (sensor_msgs/PointCloud2): 全局点云地图
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): 初始位姿

#### 发布话题
- `/localization/pose` (geometry_msgs/PoseStamped): 2D定位位姿
- `/localization/odom` (nav_msgs/Odometry): 里程计信息
- `/aligned_scan` (sensor_msgs/PointCloud2): 对齐后的扫描点云
- `/localization/fitness_score` (std_msgs/Float64): 匹配适应度分数

### TF变换
- `map` → `base_link`: 机器人在地图中的位姿

## 服务接口
- `/map_loader/reload_map` (std_srvs/Empty): 重新加载地图

## 故障排除

### 常见问题

#### 1. 地图加载失败
```bash
# 检查地图文件是否存在
ls -la ~/indoor_maps/indoor_map.pcd

# 检查文件格式
file ~/indoor_maps/indoor_map.pcd
```

#### 2. 定位漂移
- 检查初始位姿设置是否准确
- 调整匹配算法参数
- 确认激光数据质量

#### 3. 匹配失败
```bash
# 检查激光话题
rostopic hz /scan
rostopic echo /scan | head -20

# 检查地图话题
rostopic hz /global_map
```

#### 4. TF变换问题
```bash
# 查看TF树
rosrun tf view_frames

# 检查TF变换
rosrun tf tf_echo map base_link
```

### 调试命令
```bash
# 查看所有相关话题
rostopic list | grep -E "(scan|map|pose|localization)"

# 监控节点状态
rosnode list
rosnode info /map_loader
rosnode info /scan_matching_localizer

# 查看参数
rosparam list | grep -E "(map_loader|scan_matching_localizer)"
```

## 性能优化

### 1. 地图优化
- 调整`voxel_size`参数减少地图点数
- 启用`filter_outliers`去除噪声点
- 使用合适的地图分辨率

### 2. 匹配算法选择
- **ICP**：适用于结构化室内环境，精度高但计算量大
- **NDT**：适用于大范围环境，计算效率高但精度略低

### 3. Livox MID-360参数调优
```bash
# Livox MID-360室内环境推荐参数
# ICP: max_distance=1.0-2.0, max_iterations=30-50 (适合高精度定位)
# NDT: resolution=1.0-2.0, step_size=0.1-0.2 (适合实时定位)
# 扫描范围: max_range=50-100 (Livox探测距离远)
# 点云转换: min_height=-0.5, max_height=2.0 (过滤地面和天花板)
```

#### Livox专用优化设置
```bash
# 修改launch文件中的pointcloud_to_laserscan参数
<param name="min_height" value="-0.3"/>      <!-- 适合室内地面高度 -->
<param name="max_height" value="1.5"/>       <!-- 适合室内天花板高度 -->
<param name="angle_increment" value="0.0087"/>  <!-- 0.5度，平衡精度和性能 -->
<param name="range_max" value="50.0"/>       <!-- Livox室内有效距离 -->

# 针对Livox的NDT参数优化
<param name="ndt_resolution" value="1.5"/>   <!-- 适合Livox点云密度 -->
<param name="ndt_step_size" value="0.15"/>   <!-- 平衡收敛速度和精度 -->
```

## 集成导航系统

### 与move_base集成
```xml
<!-- 在你的导航launch文件中添加 -->
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <remap from="odom" to="/localization/odom"/>
    <!-- 其他move_base配置 -->
</node>
```

## 许可证
MIT License

## 作者
Your Name

## 更新日志
- v1.0.0: 初始版本，支持ICP和NDT匹配算法
