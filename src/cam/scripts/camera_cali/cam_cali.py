import cv2
import numpy as np
import glob

# 标定板参数设置
CHECKERBOARD = (8, 11)  # 棋盘格内角点数量（行列数-1）
SQUARE_SIZE = 15       # 棋盘格方块实际尺寸（毫米）

# 准备世界坐标系中的3D点
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# 存储3D点和2D点的列表
objpoints = []  # 世界坐标系中的3D点
imgpoints = []  # 图像坐标系中的2D点

# 获取所有标定图像
images = glob.glob('/home/xxx/LidarDronevoid/src/cam/calibration_images/*.jpg')  # 标定图像存储路径

# 遍历所有图像进行角点检测
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    if ret:
        objpoints.append(objp)
        
        # 亚像素级精确化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        
        # 绘制并显示角点
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Corners Found', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# 执行相机标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

# 输出焦距参数
print("\n相机内参矩阵:")
print(mtx)
print(f"\n焦距 (fx, fy): ({mtx[0,0]:.2f}, {mtx[1,1]:.2f}) pixels")
print(f"主点坐标 (cx, cy): ({mtx[0,2]:.2f}, {mtx[1,2]:.2f}) pixels")

# 计算标定误差
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print(f"\n总重投影误差: {mean_error/len(objpoints):.2f} pixels")
