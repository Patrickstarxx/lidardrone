#include <ros/ros.h>
#include <msg/ArucoMarkerArray.h>
#include <unordered_map>
// 创建快速查询字典
std::unordered_map<int, geometry_msgs::Vector3> marker_map;    

void callback(const msg::ArucoMarkerArray::ConstPtr& msg)
{
    // 构建ID-位置映射
    for(const auto& marker : msg->markers)
     {
        if(marker_map.find(marker.id) != marker_map.end()) 
        {
            ROS_WARN("检测到重重复ID: %d", marker.id);
            continue;
        }
        marker_map[marker.id] = marker.position;
        ROS_INFO("x=%f,y=%f",marker_map[1].x,marker_map[1].y);  
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_test");
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    sub_ = nh_.subscribe("/aruco_detection/markers", 10, callback);
/*     while(ros::ok())
    {
      ROS_INFO("id3.x=%f,id3.y=%f",marker_map[3].x,marker_map[3].y);  
    } */
    
    ros::spin();
    return 0;
}
