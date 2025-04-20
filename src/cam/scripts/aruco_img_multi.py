#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from msg.msg import ArucoMarkerArray, ArucoMarker  # 自定义消息

class ArucoLandingSystem:
    def __init__(self):
        rospy.init_node('aruco_landing_system', anonymous=True)

        # 摄像头参数
        self.camera_matrix = np.array([
            [341.91,   0.0,  326.11],
            [0.0,    341.07, 240.98],
            [0.0,      0.0,    1.0 ]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.0032, 0.0714, -0.0003, 0.0018, 0.0])

        # ArUco参数
        self.marker_length = 0.13
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            cv2.aruco.DetectorParameters()
        )

        # 坐标系配置
        self.R_cam_to_body = np.array([
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]])
        self.T_cam_to_body = np.array([0.06, -0.05, 0.0])

        # ROS配置
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.markers_pub = rospy.Publisher('/aruco_detection/markers', ArucoMarkerArray, queue_size=10)  # 修改发布者
        self.image_pub = rospy.Publisher('/aruco_detection/image', Image, queue_size=1)
        
        self.bridge = CvBridge()
        self.current_pose = PoseStamped().pose
        self.R_body = np.eye(3)

        # 摄像头初始化
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头！")
            raise IOError("Camera initialization failed")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        q = msg.pose.orientation
        self.R_body = tf_trans.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    def calculate_relative_position(self, tvec_cam):
        tvec_body = self.R_cam_to_body @ tvec_cam.reshape(3,1) + self.T_cam_to_body.reshape(3,1)
        tvec_world = self.R_body @ tvec_body
        scale_factor = self.current_pose.position.z / tvec_cam[2]
        return (tvec_world * scale_factor).flatten()

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            if not ret:
                continue

            # ArUco检测
            corners, ids, _ = self.detector.detectMarkers(frame)

            marker_array = ArucoMarkerArray()  # 创建消息数组
            marker_array.header.stamp = rospy.Time.now()

            if ids is not None:
                for i in range(len(ids)):
                    # 估计单个标记位姿
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], 
                        self.marker_length,
                        self.camera_matrix,
                        self.dist_coeffs
                    )
                    tvec_cam = tvec[0][0]
                    world_pos = self.calculate_relative_position(tvec_cam)

                    # 填充单个标记信息
                    marker = ArucoMarker()
                    marker.id = int(ids[i][0])
                    marker.position.x = world_pos[0]+self.current_pose.position.x
                    marker.position.y = world_pos[1]+self.current_pose.position.y
                    marker.position.z = self.current_pose.position.z-world_pos[2]
                    marker_array.markers.append(marker)

                    # 可视化
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    # cv2.putText(frame, f"ID:{ids[i][0]}", (10, 30+30*i), 
                    #           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    # cv2.putText(frame, f"X:{world_pos[0]:.2f}m", (100, 30+30*i),
                    #           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    # cv2.putText(frame, f"Y:{world_pos[1]:.2f}m", (250, 30+30*i),
                    #           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    cv2.putText(frame, f"ID:{ids[i][0]}", (10, 30+30*i), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    cv2.putText(frame, f"X:{marker.position.x:.2f}m", (100, 30+30*i),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    cv2.putText(frame, f"Y:{marker.position.y:.2f}m", (250, 30+30*i),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            # 发布消息
            self.markers_pub.publish(marker_array)
            #rospy.logerr("发布")
            
            # 发布图像
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header = marker_array.header
                self.image_pub.publish(ros_image)
            except Exception as e:
                rospy.logerr("图像转换失败: %s" % e)

            cv2.imshow('ArUco Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        system = ArucoLandingSystem()
        system.run()
    except rospy.ROSInterruptException:
        pass