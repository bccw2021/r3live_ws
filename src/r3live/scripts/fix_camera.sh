#!/bin/bash

# 摄像头问题诊断和修复脚本

echo "=========================================="
echo "Camera Diagnostic and Fix Script"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}1. 检查摄像头设备...${NC}"
echo "Available video devices:"
ls -la /dev/video* 2>/dev/null || echo "No video devices found"

echo ""
echo -e "${YELLOW}2. 检查摄像头权限...${NC}"
if [ -e "/dev/video1" ]; then
    echo "Current permissions for /dev/video1:"
    ls -la /dev/video1
    echo ""
    echo "Setting permissions..."
    sudo chmod 666 /dev/video1
    echo "New permissions:"
    ls -la /dev/video1
else
    echo -e "${RED}/dev/video1 not found${NC}"
fi

echo ""
echo -e "${YELLOW}3. 检查摄像头占用情况...${NC}"
if command -v lsof &> /dev/null; then
    echo "Processes using video devices:"
    sudo lsof /dev/video* 2>/dev/null || echo "No processes found using video devices"
else
    echo "lsof not available"
fi

echo ""
echo -e "${YELLOW}4. 检查摄像头支持的格式...${NC}"
if [ -e "/dev/video1" ]; then
    echo "Supported formats for /dev/video1:"
    v4l2-ctl --device=/dev/video1 --list-formats-ext 2>/dev/null || echo "Failed to query formats"
else
    echo -e "${RED}/dev/video1 not available${NC}"
fi

echo ""
echo -e "${YELLOW}5. 测试摄像头基本功能...${NC}"
if [ -e "/dev/video1" ]; then
    echo "Testing camera capture..."
    timeout 5s v4l2-ctl --device=/dev/video1 --stream-mmap --stream-count=1 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} Camera capture test successful"
    else
        echo -e "${RED}✗${NC} Camera capture test failed"
    fi
else
    echo -e "${RED}/dev/video1 not available for testing${NC}"
fi

echo ""
echo -e "${YELLOW}6. 检查 OpenCV 摄像头支持...${NC}"
python3 << 'EOF'
import cv2
import sys

print("OpenCV version:", cv2.__version__)
print("Available backends:", [cv2.videoio_registry.getBackendName(b) for b in cv2.videoio_registry.getBackends()])

# 测试不同的后端
backends_to_test = [
    (cv2.CAP_V4L2, "V4L2"),
    (cv2.CAP_GSTREAMER, "GStreamer"),
    (cv2.CAP_FFMPEG, "FFmpeg"),
    (cv2.CAP_ANY, "Any")
]

for backend, name in backends_to_test:
    print(f"\nTesting {name} backend...")
    cap = cv2.VideoCapture(1, backend)  # /dev/video1
    if cap.isOpened():
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"  ✓ {name}: {int(width)}x{int(height)} @ {fps} FPS")
        
        # 尝试读取一帧
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  ✓ Frame capture successful: {frame.shape}")
        else:
            print(f"  ✗ Frame capture failed")
    else:
        print(f"  ✗ {name}: Failed to open")
    cap.release()
EOF

echo ""
echo -e "${YELLOW}7. 建议的修复方案...${NC}"
echo ""
echo "如果摄像头仍然无法工作，请尝试以下方案："
echo ""
echo "方案 1: 重启摄像头模块"
echo "  sudo modprobe -r uvcvideo"
echo "  sudo modprobe uvcvideo"
echo ""
echo "方案 2: 检查其他视频设备"
echo "  尝试使用 /dev/video0 而不是 /dev/video1"
echo ""
echo "方案 3: 使用不同的 OpenCV 后端"
echo "  修改 simple_camera_publisher.py 中的 cv2.VideoCapture 参数"
echo ""
echo "方案 4: 检查系统日志"
echo "  dmesg | grep -i video"
echo "  dmesg | grep -i usb"
echo ""
echo "=========================================="

