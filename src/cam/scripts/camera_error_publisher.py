#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import os
import cv2
import math as m
from queue import Queue
import time
import rospy
from geometry_msgs.msg import Vector3  # 使用 Vector3 消息类型

# 初始化ROS节点
rospy.init_node('camera_error_publisher')
# 创建发布器，发布到 "/camera_displacement" 话题
camera_displacement_pub = rospy.Publisher('/camera_displacement', Vector3, queue_size=10)

def read2camera(cap, cap_usb_queue: Queue):
    _, frame = cap.read()
    if frame is None:
        return False
    else:
        cap_usb_queue.put(frame)

def blob_detect(frame, hsv_low, hsv_high):
    errcode = 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_low, hsv_high)
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # 找到最大面积的轮廓
    max = 0.0
    max_num = 0                          

    if len(contours) != 0:
        if len(contours) > 1:  # 找到最大的轮廓
            for i in range(len(contours)):
                c = cv2.contourArea(contours[i])
                if c > max:
                    max = c
                    max_num = i
        if cv2.contourArea(contours[max_num]) < 100:
            errcode = 2
        else:
            M = cv2.moments(contours[max_num])  # 计算轮廓的矩
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                err_x = cy - 480 * 0.5
                err_y = cx - 640 * 0.5

                # 发布err_x和err_y到"/camera_displacement"话题
                displacement = Vector3()  # 创建Vector3消息
                displacement.x = err_x
                displacement.y = err_y
                displacement.z = 0  # z方向可以设置为0
                camera_displacement_pub.publish(displacement)

                # 在图像上绘制中心和轮廓
                frame = cv2.circle(frame, (cx, cy), 8, (255, 255, 255), -1)
                frame = cv2.drawContours(frame, contours[max_num], -1, (0, 255, 255), 3)
                
            else:
                errcode = 3
    else:
        errcode = 1
    
    return errcode 

lower_red = (0, 119, 178)
upper_red = (21, 208, 255)

try:
    cap = cv2.VideoCapture(0)
    print("Usb Cam is Up")

    i = 0
    while True:
        start = time.time()
        ret, frame = cap.read(2)
        if frame is None:
            print("No stream")
            break
        elif i < 5:
            i = i + 1
            
        if blob_detect(frame, lower_red, upper_red) != 0:
            pass
        
        # 计算FPS
        end = time.time()
        inf_end = end - start
        fps = 1 / inf_end
        fps_label = "FPS: %.2f" % fps
        cv2.putText(frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow('result', frame)
        if cv2.waitKey(1) > -1:
            print("finished by user")
            break

finally:
    cap.release()
    cv2.destroyAllWindows()

