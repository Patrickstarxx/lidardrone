# -*- coding:utf-8 -*-
 
import cv2
import numpy as np
 
"""
function：读取一张图片，显示出来，转化为HSV色彩空间，并通过滑块调节HSV阈值，实时显示
"""
 
# 下面几个函数，写得有点冗余
def h_low(value):
    hsv_low[0] = value

def h_high(value):
    hsv_high[0] = value

def s_low(value):
    hsv_low[1] = value

def s_high(value):
    hsv_high[1] = value

def v_low(value):
    hsv_low[2] = value

def v_high(value):
    hsv_high[2] = value
 
try:
    
    cap = cv2.VideoCapture(0)#cv2.imread('picture.jpg') # 根据路径读取一张图片，opencv读出来的是BGR模式\
    print("Usb Cam is Up")
    
    cv2.namedWindow('dst', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('dst', 640, 480)
    #port=open_portal()

    cv2.namedWindow('BGR', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('BGR', 640, 480)

    cv2.namedWindow('image',cv2.WINDOW_AUTOSIZE)

    hsv_low = np.array([0, 0, 0])
    hsv_high = np.array([179, 255, 255])
    
    #H low：
    #    0：指向整数变量的可选指针，该变量的值反映滑块的初始位置。
    #  179：表示滑块可以达到的最大位置的值为179，最小位置始终为0。
    #h_low：指向每次滑块更改位置时要调用的函数的指针，指针指向h_low元组，有默认值0。
    cv2.createTrackbar('H low', 'image', 0, 179, h_low) 
    cv2.createTrackbar('H high', 'image', 0, 179, h_high)
    cv2.createTrackbar('S low', 'image', 0, 255, s_low)
    cv2.createTrackbar('S high', 'image', 0, 255, s_high)
    cv2.createTrackbar('V low', 'image', 0, 255, v_low)
    cv2.createTrackbar('V high', 'image', 0, 255, v_high)
 
    while True:
        ret, image = cap.read()
        if image is None:
            print("No stream")
            break
        cv2.imshow("BGR", image)  #正常读取显示图像帧

        dst = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # BGR转HSV
        dst = cv2.inRange(dst, hsv_low, hsv_high) # 通过HSV的高低阈值，提取图像部分区域
        cv2.imshow('dst', dst)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
        #if cv2.waitKey(1) == ord('s'):
        #    print(hsv_low,hsv_high)
        #    cv2.imwrite('HSV_pic.jpg',dst)
finally:
    #realsense_camera.stop_pipeline()
    pass