#include <ros/ros.h>
#include <msg/ArucoMarkerArray.h>
#include <unordered_map>
#include <geometry_msgs/Vector3.h>
// 创建快速查询字典
std::unordered_map<int, geometry_msgs::Vector3> marker_map;    
char ids[10];
char id_index=0;
char dog_sent=0;
void callback(const msg::ArucoMarkerArray::ConstPtr& msg)
{

    bool any_new = false;     // 是否发现新ID
    
    // 遍历所有检测到的标记
    for(const auto& marker : msg->markers)
    {
        // 尝试查找ID
        auto it = marker_map.find(marker.id);
        
        if(it == marker_map.end()) 
        {
            // 新ID处理流程
            marker_map.emplace(marker.id, marker.position);  // 高效插入方式
            ROS_INFO_STREAM("新增ID:" << marker.id 
                          << " 坐标(" << marker.position.x 
                          << ", " << marker.position.y << ")");
            any_new = true;
            ids[id_index]=marker.id;
            ROS_INFO("index=%d",id_index);
            ROS_INFO("ids[index]=%d",ids[id_index]);
            id_index++;
        }
        else 
        {
            // 已存在ID可选的更新逻辑
            // it->second = marker.position;  // 如果需要更新位置
            ROS_WARN("ID exist:%d",marker.id);
        }
        //ROS_INFO_STREAM("ID 44 x"<<marker_map[44].x<<" y"<<marker_map[44].y);

        if(dog_sent!=id_index)
        {
            //ROS_INFO("if");
            char dog_to_send=id_index-dog_sent;
            for(int i=0;i<dog_to_send;i++)
            {
                //ROS_INFO("for");
                marker_map[ids[id_index-1-i]].z=ids[id_index-1-i];
                //dog_pub.publish(marker_map[ids[id_index-1-i]]);
                ROS_WARN("id=%f,x=%f,y=%f",marker_map[ids[id_index-1-i]].z,marker_map[ids[id_index-1-i]].x,marker_map[ids[id_index-1-i]].y);
                dog_sent++;
            }
        }
    }


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_test");
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Rate rate(20.0);// 50ms
    
	ros::Publisher dog_pub = nh_.advertise<geometry_msgs::Vector3>("/dog_target",10);
    sub_ = nh_.subscribe("/aruco_relative_position", 10, callback);
    //ROS_INFO("while");
/*     ROS_INFO("dog_sent=%d",dog_sent);
    ROS_INFO("id_index=%d",id_index); */


    
   
  
    // ros::spinOnce();
    // rate.sleep();
    ros::spin();
    return 0;
}
