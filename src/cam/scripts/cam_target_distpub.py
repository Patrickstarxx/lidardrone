#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class CameraDistanceEstimator:
    def __init__(self):
        # ROS初始化
        rospy.init_node('camera_distance_publisher')
        
        # 相机参数（根据实际标定结果修改）
        self.fx = 448   # X轴焦距 (pixels)
        self.fy = 448   # Y轴焦距 (pixels)
        self.image_center = (328, 255)  # 图像中心 (cx, cy)
        
        # 高度数据 (利用浮点赋值原子性)
        self.current_height = 0.76  # 默认高度 (米)
        
        # 初始化ROS通信
        self.pub = rospy.Publisher('/camera_displacement', Vector3, queue_size=1)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, 
                        self.height_callback, queue_size=1)
        
        # 视频捕获
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头!")
            return

        # 颜色阈值
        """self.lower_red = (0, 119, 178)
        self.upper_red = (21, 208, 255) """

        self.lower_red = (0, 23, 137)
        self.upper_red = (17, 161, 239) 

    def height_callback(self, msg):
        """高度数据回调函数"""
        # 直接赋值原子操作（确保z是基本数据类型）
        self.current_height = msg.pose.pose.position.z

    def pixel_to_distance(self, cx, cy):
        """将像素坐标转换为实际距离"""
        # 高度安全检查
        if self.current_height < 0.1:
            rospy.logwarn_throttle(1, "低高度警告: %.2f m", self.current_height)
            return 0.0, 0.0
        
        # 使用临时变量确保计算过程的一致性
        h = self.current_height
        dx = cx - self.image_center[0]
        dy = cy - self.image_center[1]
        
        return (dx * h / self.fx, dy * h / self.fy)

    def detect_blob(self, frame):
        """检测色块并返回距离信息"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None
        
        # 寻找最大有效轮廓
        max_contour = max(contours, key=lambda c: cv2.contourArea(c))
        if cv2.contourArea(max_contour) < 100:
            return None, None
        
        # 计算质心
        M = cv2.moments(max_contour)
        if M["m00"] == 0:
            return None, None
            
        return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

    def run(self):
        rate = rospy.Rate(30)  # 与摄像头帧率同步
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn_throttle(5, "摄像头信号丢失")
                continue
            
            # 检测色块中心
            cx, cy = self.detect_blob(frame)
            
            if cx is not None:
                # 获取实时高度快照
                h = self.current_height
                
                # 计算实际距离
                dist_x = (cx - self.image_center[0]) * h / self.fx
                dist_y = (cy - self.image_center[1]) * h / self.fy
                
                # 发布消息
                self.pub.publish(Vector3(x=dist_y, y=dist_x, z=h))
                
                # 绘制界面
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(frame, f"X:{dist_y:.2f}m Y:{dist_x:.2f}m", (10,30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.putText(frame, f"Height:{h:.2f}m", (10,60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            
            # 显示结果
            cv2.imshow("Distance Estimation", frame)
            if cv2.waitKey(1) == 27:
                break
            
            rate.sleep()
        
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        estimator = CameraDistanceEstimator()
        estimator.run()
    except rospy.ROSInterruptException:
        pass