#!/usr/bin/env python3
"""
Simple Camera Publisher Node
发布摄像头图像到 /usb_cam/image_raw 话题，替代 usb_cam 包
适用于 R3LIVE 系统测试
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import yaml
import os

class SimpleCameraPublisher:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('simple_camera_publisher', anonymous=True)
        
        # 获取参数
        self.video_device = rospy.get_param('~video_device', '/dev/video1')
        self.frame_rate = rospy.get_param('~framerate', 30)
        self.image_width = rospy.get_param('~image_width', 1280)
        self.image_height = rospy.get_param('~image_height', 1024)
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'usb_cam')
        self.camera_name = rospy.get_param('~camera_name', 'r3live_camera')
        self.camera_info_url = rospy.get_param('~camera_info_url', '')
        
        # 创建发布器
        self.image_pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/usb_cam/camera_info', CameraInfo, queue_size=10)
        
        # 创建 CV Bridge
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.cap = None
        self.camera_info = None
        
        # 加载相机标定信息
        self.load_camera_info()
        
        # 设置发布频率
        self.rate = rospy.Rate(self.frame_rate)
        
        rospy.loginfo(f"Simple Camera Publisher initialized")
        rospy.loginfo(f"Video device: {self.video_device}")
        rospy.loginfo(f"Frame rate: {self.frame_rate} Hz")
        rospy.loginfo(f"Resolution: {self.image_width}x{self.image_height}")
    
    def load_camera_info(self):
        """加载相机标定信息"""
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.camera_frame_id
        self.camera_info.width = self.image_width
        self.camera_info.height = self.image_height
        
        # 尝试从文件加载标定参数
        if self.camera_info_url and self.camera_info_url.startswith('package://'):
            # 解析 package:// URL
            package_path = self.camera_info_url.replace('package://r3live/', '')
            config_path = os.path.expanduser('~/r3live_ws/src/r3live/') + package_path
            
            if os.path.exists(config_path):
                try:
                    with open(config_path, 'r') as f:
                        camera_data = yaml.safe_load(f)
                    
                    if 'camera_matrix' in camera_data:
                        self.camera_info.K = camera_data['camera_matrix']['data']
                    if 'distortion_coefficients' in camera_data:
                        self.camera_info.D = camera_data['distortion_coefficients']['data']
                    if 'rectification_matrix' in camera_data:
                        self.camera_info.R = camera_data['rectification_matrix']['data']
                    if 'projection_matrix' in camera_data:
                        self.camera_info.P = camera_data['projection_matrix']['data']
                    
                    self.camera_info.distortion_model = camera_data.get('distortion_model', 'plumb_bob')
                    
                    rospy.loginfo(f"Loaded camera calibration from: {config_path}")
                except Exception as e:
                    rospy.logwarn(f"Failed to load camera calibration: {e}")
                    self.set_default_camera_info()
            else:
                rospy.logwarn(f"Camera info file not found: {config_path}")
                self.set_default_camera_info()
        else:
            self.set_default_camera_info()
    
    def set_default_camera_info(self):
        """设置默认的相机标定参数"""
        # 使用 R3LIVE 配置文件中的参数
        self.camera_info.K = [
            863.424, 0.0, 640.681,
            0.0, 863.417, 518.339,
            0.0, 0.0, 1.0
        ]
        
        self.camera_info.D = [-0.108, 0.105, -0.00012872, 5.7923e-05, -0.0222]
        
        self.camera_info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        self.camera_info.P = [
            863.424, 0.0, 640.681, 0.0,
            0.0, 863.417, 518.339, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.camera_info.distortion_model = 'plumb_bob'
        rospy.loginfo("Using default camera calibration parameters")
    
    def initialize_camera(self):
        """初始化摄像头"""
        try:
            # 直接使用 V4L2 后端（已验证有效）
            device_id = int(self.video_device.split('video')[-1]) if 'video' in self.video_device else 0
            
            rospy.loginfo(f"Opening camera with V4L2 backend...")
            self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                rospy.logerr(f"Failed to open camera device: {self.video_device}")
                return False
            
            # 设置摄像头参数
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            # 验证设置
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            rospy.loginfo(f"Camera initialized successfully")
            rospy.loginfo(f"Actual resolution: {actual_width}x{actual_height}")
            rospy.loginfo(f"Actual FPS: {actual_fps}")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error initializing camera: {e}")
            return False
    
    def publish_frame(self, frame):
        """发布图像帧和相机信息"""
        try:
            # 获取当前时间戳
            current_time = rospy.Time.now()
            
            # 转换图像格式并发布
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = current_time
            img_msg.header.frame_id = self.camera_frame_id
            self.image_pub.publish(img_msg)
            
            # 发布相机信息
            self.camera_info.header.stamp = current_time
            self.camera_info_pub.publish(self.camera_info)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error publishing frame: {e}")
    
    def run(self):
        """主运行循环"""
        # 初始化摄像头
        if not self.initialize_camera():
            rospy.logerr("Failed to initialize camera. Exiting.")
            return
        
        rospy.loginfo("Starting camera publishing loop...")
        
        frame_count = 0
        last_info_time = rospy.Time.now()
        
        try:
            while not rospy.is_shutdown():
                # 读取帧
                ret, frame = self.cap.read()
                
                if ret:
                    # 调整图像尺寸（如果需要）
                    if frame.shape[1] != self.image_width or frame.shape[0] != self.image_height:
                        frame = cv2.resize(frame, (self.image_width, self.image_height))
                    
                    # 发布帧
                    self.publish_frame(frame)
                    
                    frame_count += 1
                    
                    # 每 5 秒打印一次状态信息
                    current_time = rospy.Time.now()
                    if (current_time - last_info_time).to_sec() >= 5.0:
                        rospy.loginfo(f"Published {frame_count} frames")
                        last_info_time = current_time
                        frame_count = 0
                
                else:
                    rospy.logwarn("Failed to read frame from camera")
                    rospy.sleep(0.1)  # 短暂等待后重试
                
                # 控制发布频率
                self.rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Received interrupt signal")
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("Cleaning up camera resources...")
        if self.cap is not None:
            self.cap.release()
        rospy.loginfo("Camera publisher shutdown complete")

def main():
    try:
        camera_publisher = SimpleCameraPublisher()
        camera_publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera publisher interrupted")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    main()

