#! /usr/bin/env python3
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
class ArucoDroneLanding:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('aruco_drone_landing', anonymous=True)
        
        # 订阅无人机姿态
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        # 发布二维码中心相对于相机中心的水平距离
        self.pub = rospy.Publisher('/aruco_horizontal_distance', Vector3, queue_size=10)
        # 初始化变量
        self.h = 1
        
        # 打开USB摄像头
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 480)  
        self.cap.set(4, 640)  
        if not self.cap.isOpened():
            raise IOError("Cannot open USB CAM")
        else:
        	rospy.logerr('USB CAM is UP')
        
        # 设置相机内参（这里假设默认值，实际应用中需要根据你的摄像头进行校准）
        self.image_center = (240, 320)  # 图像中心 (cx, cy)
        self.fx=376.95550496
        self.fy=375.83052548
        self.camera_matrix = np.array([ [341.91275249, 0, 326.11605895],
        				                [0, 341.07830571, 240.98064699],
    	        			            [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))  # 假设畸变系数为零
        
        # Aruco字典
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        
    def pose_callback(self, msg):
        self.h = msg.pose.position.z  # 获取无人机高度
    
    def run(self):
        while not rospy.is_shutdown() and self.cap.isOpened():
            ret, frame = self.cap.read()
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)#顺时针270度旋转
            if not ret:
                break
            
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
            
            if len(corners) > 0:
                
                for i in range(len(ids)):
                    corner = corners[i][0]
                    center_pixel = np.mean(corner, axis=0)

                    dist_x = (center_pixel[0] - self.image_center[0]) * self.h / self.fx
                    dist_y = (center_pixel[1] - self.image_center[1]) * self.h / self.fy
                    
                    # 创建并发布消息
                    self.pub.publish(Vector3(x=-dist_y, y=-dist_x, z=self.h))
                    # 绘制二维码边界和轴
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.putText(frame, f"X:{-dist_y:.2f}m Y:{-dist_x:.2f}m", (10,30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    #cv2.circle(frame, self.image_center, 20, (255,0,0), -1)
                    cv2.putText(frame, f"Height:{self.h:.2f}m", (10,60),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        aruco_drone_landing = ArucoDroneLanding()
        aruco_drone_landing.run()
    except rospy.ROSInterruptException:
        pass




