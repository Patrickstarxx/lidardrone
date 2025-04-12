import cv2
import numpy as np
import os
from datetime import datetime

# 配置参数
CHECKERBOARD = (8, 11)        # 棋盘格内角点数（行列数-1）
SAVE_FOLDER = "/home/xxx/LidarDronevoid/src/cam/calibration_images"  # 保存路径
CAMERA_ID = 0                # 摄像头设备号

# 创建保存目录
os.makedirs(SAVE_FOLDER, exist_ok=True)

# 初始化摄像头
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    print("无法打开摄像头！")
    exit()

print("摄像头已启动，操作指南：")
print("- 按 SPACE 键 保存当前帧")
print("- 按 ESC 键 退出程序")

count = 0  # 已保存图片计数器

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法获取视频流")
        break
    
    # 显示实时画面
    display = frame.copy()
    
    # 在画面上添加操作提示
    cv2.putText(display, f"Captured: {count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(display, "Press SPACE to capture", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(display, "Press ESC to exit", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    cv2.imshow("Camera Calibration", display)
    
    # 按键处理
    key = cv2.waitKey(1)
    if key == 27:  # ESC 键退出
        break
    elif key == 32:  # 空格键保存
        # 生成唯一文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(SAVE_FOLDER, f"calib_{timestamp}.jpg")
        
        # 保存图片
        cv2.imwrite(filename, frame)
        count += 1
        print(f"已保存: {filename}")
        
        # 显示保存反馈
        cv2.putText(display, "CAPTURED!", (200, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
        cv2.imshow("Camera Calibration", display)
        cv2.waitKey(300)  # 显示反馈0.3秒

# 释放资源
cap.release()
cv2.destroyAllWindows()
print(f"\n标定图像采集完成，共保存 {count} 张图片到 {SAVE_FOLDER} 文件夹")
