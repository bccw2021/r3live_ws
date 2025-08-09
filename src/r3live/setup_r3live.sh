#!/bin/bash

# R3LIVE 快速设置脚本
# 解决文件权限和路径问题

echo "=========================================="
echo "R3LIVE Quick Setup Script"
echo "=========================================="

# 设置颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 设置 Python 脚本权限
echo -e "${YELLOW}1. 设置 Python 脚本权限...${NC}"
chmod +x ~/r3live_ws/src/r3live/scripts/simple_camera_publisher.py
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Python 脚本权限设置成功"
else
    echo -e "${RED}✗${NC} Python 脚本权限设置失败"
fi

# 2. 创建符号链接到 devel/lib/r3live/ 目录
echo -e "${YELLOW}2. 创建 Python 脚本符号链接...${NC}"
mkdir -p ~/r3live_ws/devel/lib/r3live/
ln -sf ~/r3live_ws/src/r3live/scripts/simple_camera_publisher.py ~/r3live_ws/devel/lib/r3live/simple_camera_publisher.py
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} Python 脚本符号链接创建成功"
else
    echo -e "${RED}✗${NC} Python 脚本符号链接创建失败"
fi

# 3. 编译 R3LIVE
echo -e "${YELLOW}3. 编译 R3LIVE...${NC}"
cd ~/r3live_ws
catkin_make
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} R3LIVE 编译成功"
else
    echo -e "${RED}✗${NC} R3LIVE 编译失败"
fi

# 4. 检查可执行文件
echo -e "${YELLOW}4. 检查可执行文件...${NC}"
if [ -f "~/r3live_ws/devel/lib/r3live/r3live_mapping" ]; then
    echo -e "${GREEN}✓${NC} r3live_mapping 可执行文件存在"
else
    echo -e "${RED}✗${NC} r3live_mapping 可执行文件缺失"
fi

if [ -f "~/r3live_ws/devel/lib/r3live/r3live_LiDAR_front_end" ]; then
    echo -e "${GREEN}✓${NC} r3live_LiDAR_front_end 可执行文件存在"
else
    echo -e "${RED}✗${NC} r3live_LiDAR_front_end 可执行文件缺失"
fi

if [ -f "~/r3live_ws/devel/lib/r3live/simple_camera_publisher.py" ]; then
    echo -e "${GREEN}✓${NC} simple_camera_publisher.py 链接存在"
else
    echo -e "${RED}✗${NC} simple_camera_publisher.py 链接缺失"
fi

# 5. 显示启动命令
echo ""
echo "=========================================="
echo -e "${GREEN}设置完成！${NC}"
echo ""
echo "现在你可以启动 R3LIVE："
echo ""
echo -e "${YELLOW}cd ~/r3live_ws${NC}"
echo -e "${YELLOW}source devel/setup.bash${NC}"
echo -e "${YELLOW}roslaunch r3live r3live_bag.launch${NC}"
echo ""
echo "=========================================="

