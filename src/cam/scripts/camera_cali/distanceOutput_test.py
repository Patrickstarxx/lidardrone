import cv2
import numpy as np

class DistanceEstimator:
    def __init__(self, fx=800, fy=800, cx=320, cy=240):
        # 相机内参（需根据实际标定结果修改）
        self.fx = fx    # X轴焦距 (pixels)
        self.fy = fy    # Y轴焦距 (pixels)
        self.cx = cx    # 主点X坐标
        self.cy = cy    # 主点Y坐标
        
        # 初始化参数
        self.Z = 0.5    # 初始高度 (米)
        self.click_pos = None  # 鼠标点击位置
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)
        cv2.namedWindow("Distance Estimation")
        cv2.setMouseCallback("Distance Estimation", self.mouse_callback)
        
        # 创建高度调节滑块
        cv2.createTrackbar("Height (m)", "Distance Estimation", 10, 100, self.on_trackbar)
        self.update_height(1.0)  # 初始高度1米

    def update_height(self, value):
        """更新高度值（滑块值除以10获得米单位）"""
        self.Z = max(0.1, value / 10.0)  # 限制最小高度0.1米

    def on_trackbar(self, val):
        """滑条回调函数"""
        self.update_height(val)

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标点击回调"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_pos = (x, y)

    def pixel_to_real(self, u, v):
        """将像素坐标转换为实际坐标"""
        # 计算相对于图像中心的偏移量
        du = u - self.cx
        dv = v - self.cy
        
        # 计算实际坐标 (米)
        X = (du / self.fx) * self.Z
        Y = (dv / self.fy) * self.Z
        return X, Y

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # 显示参考坐标系
            cv2.drawMarker(frame, (self.cx, self.cy), (0,255,0), 
                           cv2.MARKER_CROSS, 20, 2)
            
            # 如果有点击位置，计算并显示坐标
            if self.click_pos:
                u, v = self.click_pos
                X, Y = self.pixel_to_real(u, v)
                
                # 绘制点击位置
                cv2.circle(frame, (u,v), 8, (0,0,255), -1)
                
                # 显示坐标信息
                text = f"X:{X:.2f}m, Y:{Y:.2f}m (Z={self.Z:.1f}m)"
                cv2.putText(frame, text, (10,30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
            
            # 显示界面
            cv2.imshow("Distance Estimation", frame)
            
            # 退出键处理
            key = cv2.waitKey(1)
            if key == 27:  # ESC退出
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 使用方法：
    # 1. 修改fx,fy,cx,cy为你的相机标定结果
    # 2. 运行程序后：
    #    - 用鼠标点击图像任意位置
    #    - 调节上方滑块设置高度
    #    - 按ESC退出
    
    estimator = DistanceEstimator(
        fx=448,   # 替换为你的实际焦距
        fy=448,   # 替换为你的实际焦距
        cx=328,   # 替换为你的主点坐标
        cy=255    # 替换为你的主点坐标
    )
    estimator.run()
