#!/bin/bash

# R3LIVE 系统测试脚本
# 用于验证 R3LIVE 在 RK3588 上的完整功能

echo "=========================================="
echo "R3LIVE System Test Script for RK3588"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1 is available"
        return 0
    else
        echo -e "${RED}✗${NC} $1 is not available"
        return 1
    fi
}

check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} File exists: $1"
        return 0
    else
        echo -e "${RED}✗${NC} File missing: $1"
        return 1
    fi
}

check_device() {
    if [ -e "$1" ]; then
        echo -e "${GREEN}✓${NC} Device exists: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Device missing: $1"
        return 1
    fi
}

echo ""
echo "1. 检查基础环境..."
echo "-------------------"

# 检查 ROS 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗${NC} ROS environment not sourced"
    echo "Please run: source /opt/ros/noetic/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓${NC} ROS $ROS_DISTRO environment loaded"
fi

# 检查工作空间
if [ -f "~/r3live_ws/devel/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} R3LIVE workspace found"
else
    echo -e "${RED}✗${NC} R3LIVE workspace not found or not built"
fi

echo ""
echo "2. 检查必要的命令工具..."
echo "------------------------"

check_command "roscore"
check_command "roslaunch"
check_command "rostopic"
check_command "python3"
check_command "v4l2-ctl"

echo ""
echo "3. 检查摄像头设备..."
echo "-------------------"

check_device "/dev/video0"
check_device "/dev/video1"

# 检查摄像头详细信息
if [ -e "/dev/video1" ]; then
    echo ""
    echo "摄像头设备信息:"
    v4l2-ctl --device=/dev/video1 --list-formats-ext 2>/dev/null | head -10
fi

echo ""
echo "4. 检查 R3LIVE 文件..."
echo "---------------------"

# 检查关键文件
check_file "~/r3live_ws/src/r3live/r3live/launch/r3live_bag.launch"
check_file "~/r3live_ws/src/r3live/r3live/launch/simple_camera.launch"
check_file "~/r3live_ws/src/r3live/scripts/simple_camera_publisher.py"
check_file "~/r3live_ws/src/r3live/config/camera_info.yaml"
check_file "~/r3live_ws/src/r3live/config/r3live_config.yaml"

# 检查可执行文件
echo ""
echo "检查 R3LIVE 可执行文件:"
if [ -f "~/r3live_ws/devel/lib/r3live/r3live_mapping" ]; then
    echo -e "${GREEN}✓${NC} r3live_mapping executable found"
else
    echo -e "${RED}✗${NC} r3live_mapping executable missing"
fi

if [ -f "~/r3live_ws/devel/lib/r3live/r3live_LiDAR_front_end" ]; then
    echo -e "${GREEN}✓${NC} r3live_LiDAR_front_end executable found"
else
    echo -e "${YELLOW}!${NC} r3live_LiDAR_front_end executable missing (may need compilation)"
fi

echo ""
echo "5. 检查 Python 依赖..."
echo "---------------------"

python3 -c "import cv2; print('OpenCV version:', cv2.__version__)" 2>/dev/null && echo -e "${GREEN}✓${NC} OpenCV for Python available" || echo -e "${RED}✗${NC} OpenCV for Python missing"
python3 -c "import rospy; print('rospy available')" 2>/dev/null && echo -e "${GREEN}✓${NC} rospy available" || echo -e "${RED}✗${NC} rospy missing"
python3 -c "import sensor_msgs; print('sensor_msgs available')" 2>/dev/null && echo -e "${GREEN}✓${NC} sensor_msgs available" || echo -e "${RED}✗${NC} sensor_msgs missing"
python3 -c "import cv_bridge; print('cv_bridge available')" 2>/dev/null && echo -e "${GREEN}✓${NC} cv_bridge available" || echo -e "${RED}✗${NC} cv_bridge missing"

echo ""
echo "6. 系统架构信息..."
echo "-----------------"

echo "Architecture: $(uname -m)"
echo "Kernel: $(uname -r)"
echo "OS: $(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown')"

# 检查 CPU 信息
if [ -f "/proc/cpuinfo" ]; then
    cpu_model=$(grep "model name" /proc/cpuinfo | head -1 | cut -d: -f2 | xargs)
    echo "CPU: $cpu_model"
fi

echo ""
echo "=========================================="
echo "测试完成！"
echo ""
echo "如果所有检查都通过，你可以运行以下命令启动 R3LIVE："
echo ""
echo -e "${YELLOW}# 确保脚本有执行权限${NC}"
echo "chmod +x ~/r3live_ws/src/r3live/scripts/simple_camera_publisher.py"
echo ""
echo -e "${YELLOW}# 启动 R3LIVE 系统${NC}"
echo "cd ~/r3live_ws"
echo "source devel/setup.bash"
echo "roslaunch r3live r3live_bag.launch"
echo ""
echo -e "${YELLOW}# 在另一个终端中检查话题${NC}"
echo "rostopic list | grep usb_cam"
echo "rostopic hz /usb_cam/image_raw"
echo ""
echo "=========================================="

