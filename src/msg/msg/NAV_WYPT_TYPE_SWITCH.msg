# Flight Control Mode (飞行控制模式)
uint8 FC_MODE_OFFB = 0    # Offboard 模式
uint8 FC_MODE_ARM  = 1    # 解锁（Arm）
uint8 FC_MODE_LAND = 2    # 降落模式

# Navigation Waypoint Type (导航航点类型)
uint8 NAV_WYPT_PRESET = 10  # 预设航点
uint8 NAV_WYPT_RVIZ   = 11  # RVIZ 设置的航点
uint8 NAV_WYPT_LAND   = 12  # 降落航点
uint8 NAV_WYPT_TAKEOFF= 13  # 起飞航点
uint8 NAV_WYPT_RETURN = 14  # 返回
uint8 NAV_WYPT_CAM    = 15  # 摄像头
uint8 nav_waypoint_type_switch