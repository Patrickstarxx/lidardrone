#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, Vector3

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
        self.marker_length = 0.2
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            cv2.aruco.DetectorParameters()
        )

        # 坐标系配置
        self.R_cam_to_body = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]])
        self.T_cam_to_body = np.array([0.0, 0.0, -0.1])

        # ROS配置
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.pos_pub = rospy.Publisher('/aruco_relative_position', Vector3, queue_size=10)
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
            frame=cv2.rotate(frame,cv2.ROTATE_180)
            if not ret:
                continue

            # ArUco检测
            corners, ids, _ = self.detector.detectMarkers(frame)
            
            if ids is not None:
                for i in range(len(ids)):
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], 
                        self.marker_length,
                        self.camera_matrix,
                        self.dist_coeffs
                    )
                    tvec_cam = tvec[0][0]
                    world_pos = self.calculate_relative_position(tvec_cam)

                    msg = Vector3()
                    msg.x = -world_pos[0]
                    msg.y = -world_pos[1]
                    msg.z = self.current_pose.position.z
                    self.pos_pub.publish(msg)

                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    cv2.putText(frame, f"X:{-world_pos[0]:.2f}m", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    cv2.putText(frame, f"Y:{-world_pos[1]:.2f}m", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

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