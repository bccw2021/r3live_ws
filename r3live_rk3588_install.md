# 1.install opencv #
------

# 检查是否安装了 OpenCV
dpkg -l | grep opencv

# 检查 pkg-config 路径
echo $PKG_CONFIG_PATH

# 查找可能存在的 opencv.pc 文件
find /usr -name "opencv*.pc" 2>/dev/null



# 测试 opencv4 包
pkg-config --modversion opencv4
pkg-config --cflags --libs opencv4



# 进入 pkgconfig 目录
cd /usr/lib/aarch64-linux-gnu/pkgconfig/

# 创建 opencv.pc 符号链接指向 opencv4.pc
sudo ln -sf opencv4.pc opencv.pc

# 验证符号链接创建成功
ls -la opencv*

# 测试 opencv 包是否现在可以被找到
pkg-config --modversion opencv



# 回到 R3LIVE 目录
cd /Users/liuchuan/r3live

# 重新尝试配置或编译
# 如果使用 catkin，可能需要：
catkin_make
# 或者如果使用 cmake：
mkdir build && cd build
cmake



# 2.修复 R3LIVE 代码中的 跨架构兼容性问题，具体来说是解决 ARM64 架构（RK3588）上缺少 cpuid.h 头文件的编译错误。#
------
# 1. 备份原文件
cp tools_logger.hpp tools_logger.hpp.backup

# 2. 修复第一个 cpuid.h 包含（将 #else 改为条件编译）
sed -i '1010s/#else/#elif defined(__x86_64__) || defined(__i386__)/' tools_logger.hpp

# 3. 在第一个 #endif 前添加 ARM 实现
sed -i '1017i\\n#else\n// ARM or other architectures - provide dummy implementation\ninline void CPUID( int CPUInfo[ 4 ], int level )\n{\n    CPUInfo[0] = CPUInfo[1] = CPUInfo[2] = CPUInfo[3] = 0;\n}' tools_logger.hpp

# 4. 查看修改后的结果
head -n 1030 tools_logger.hpp | tail -n 25

# 查找第二个 cpuid.h 的位置
grep -n "cpuid.h" tools_logger.hpp

# 修复第二个 cpuid.h 包含（行号可能已经变化）
sed -i 's/#else/#elif defined(__x86_64__) || defined(__i386__)/' tools_logger.hpp

# 在对应位置添加 ARM 实现
# 这个需要根据实际行号调整


# 回到项目根目录
cd /home/cat/r3live_ws/src/r3live/r3live

# 修复 Util.cpp 中的 cpuid.h
sed -i 's/#else/#elif defined(__x86_64__) || defined(__i386__)/' src/meshing/MVS/Common/Util.cpp



# 3. hosts 文件缺少 localhost 的标准映射#
------

RLException: cannot resolve host address for machine [localhost]
The traceback for the exception was written to the log file

# 添加 localhost 映射到 hosts 文件
echo "127.0.0.1 localhost" | sudo tee -a /etc/hosts

# 验证修改
cat /etc/hosts



# 4. 怎么查看摄像头？#
------
# 查看所有视频设备
ls /dev/video*

# 查看设备详细信息
v4l2-ctl --list-devices

# 如果没有 v4l2-ctl，先安装
sudo apt install v4l-utils



# 查看摄像头支持的格式
v4l2-ctl --device=/dev/video0 --list-formats-ext

# 查看摄像头当前设置
v4l2-ctl --device=/dev/video0 --all


# 安装 fswebcam
sudo apt install fswebcam

# 拍摄一张照片测试
fswebcam -d /dev/video0 -r 640x480 test.jpg

# 查看照片
eog test.jpg  # 或者 display test.jpg


sudo apt install ros-noetic-usb-cam ros-noetic-image-view


# 查看哪些进程在使用摄像头
lsof /dev/video0

# 杀死占用摄像头的进程
sudo pkill -f "进程名"



# 查看内核消息中的摄像头信息
dmesg | grep -i camera
dmesg | grep -i video

# 查看设备树中的摄像头配置
ls /sys/class/video4linux/
cat /sys/class/video4linux/video*/name


# 检查 USB 摄像头是否被识别
lsusb | grep -i camera

# 重新加载 USB 视频驱动
sudo modprobe uvcvideo


# 添加用户到 video 组
sudo usermod -a -G video $USER

# 重新登录或者
newgrp video



# 编译 #
------
catkin_make --only-pkg-with-deps livox_ros_driver











