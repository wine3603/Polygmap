#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

def rgb_callback(msg):
    bridge = CvBridge()
    try:
        # 将ROS的图像消息转换为OpenCV图像
        color_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(f"Received RGB frame: {color_image.shape}")
    except Exception as e:
        print(f"Error converting RGB image: {e}")

def depth_callback(msg):
    bridge = CvBridge()
    try:
        # 将深度图像消息转换为NumPy数组
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        print(f"Received Depth frame: {depth_image.shape}")
    except Exception as e:
        print(f"Error converting Depth image: {e}")

def listener():
    rospy.init_node('realsense_listener', anonymous=True)
    
    # 订阅RGB和深度图像话题
    rospy.Subscriber('/l515/color/image_raw', Image, rgb_callback)
    rospy.Subscriber('/l515/depth/image_rect_raw', Image, depth_callback)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    listener()
