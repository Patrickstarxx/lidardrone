#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>

#include <vector>
#include<msg/NAV_WYPT_TYPE_SWITCH.h>
#include<msg/NAV_WYPT_MODE.h>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <limits>

#include <ros/ros.h>
#include <termios.h>    // Linux终端控制
#include <unistd.h>
#include <fcntl.h>
bool goalmaunlreach_flag=false;
bool goalmanulreached=false;
double tolerance=0.2;//设置容差
double dx,dy,dz;
bool rviz_flag=false,preset_flag=true,next_poniit_flag=false;
int num_cnt_=0;
int test_=5;
bool takeoff_done=false;
bool cam_target_reached=false;

size_t wypt_current_index_ = 0;
bool mission_start=false;
bool get_oringin_pos=false;
int IS_RVIZ_GOAL=-1;
msg::NAV_WYPT_TYPE_SWITCH NWTS;
msg::NAV_WYPT_MODE NWM;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State current_state;

geometry_msgs::PoseStamped rviz_goal;
geometry_msgs::PoseStamped _WayPoints;//导航点

geometry_msgs::TwistStamped local_vel;
geometry_msgs::PoseStamped local_pos;
geometry_msgs::PoseStamped home_pos;

geometry_msgs::Vector3 cam_target;

//订阅无人机状态
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// 订阅的无人机当前位置数据
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
	if(!get_oringin_pos)//获取初始位置
	{
		home_pos.pose.position.x=local_pos.pose.position.x;
		home_pos.pose.position.y=local_pos.pose.position.y;
		home_pos.pose.position.z=0;
		get_oringin_pos=true;
	}
}

// 訂閱RVIZ 2D NAV GOAL
void rviz_goal_cb(const geometry_msgs::PoseStampedPtr &msg)
{
	rviz_goal = *msg;
	rviz_goal.pose.position.z=1.2;//2D NAV GOAL发布的导航点默认高度为0，手动赋值避免贴地飞行
   // 更新时间戳为当前时间
   rviz_goal.header.stamp = ros::Time::now();

   if(IS_RVIZ_GOAL == -1)
   {
	IS_RVIZ_GOAL = rviz_goal.header.seq;
   }
   if(IS_RVIZ_GOAL == rviz_goal.header.seq && takeoff_done)
   {
	NWTS.nav_waypoint_type_switch = NWTS.NAV_WYPT_RVIZ;
	ROS_ERROR("IS_RVIZ=SEQ");
	rviz_flag=true;
	preset_flag=false;
   }
   
}

/**
 * @brief 按顺序读取预设导航点，数据格式与 Rviz 2D Nav Goal 完全一致
 * @return geometry_msgs::PoseStamped 
 *         无效数据时返回 header.frame_id = "invalid"
 */
geometry_msgs::PoseStamped getNextNavGoal() 
{
	static size_t index = 0;                 // 静态索引计数器
	static XmlRpc::XmlRpcValue goal_list;    // 静态数据缓存
	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "invalid";        // 默认无效标识
  
	// 首次调用时加载数据
	if (index == 0) 
	{
	  if (!ros::param::get("/waypoints", goal_list)) 
	  {
		ROS_ERROR("Failed to load navigation goals!");
		return goal;
	  }
	}
	// 遍历数据列表
	if (index < goal_list.size()) 
	{
	  try 
	  {
		// 解析 Header
		XmlRpc::XmlRpcValue& header = goal_list[index]["header"];
		goal.header.frame_id = static_cast<std::string>(header["frame_id"]);
		goal.header.stamp = ros::Time::now();  // 自动添加时间戳
  
		// 解析 Pose
		XmlRpc::XmlRpcValue& pose = goal_list[index]["pose"];
		XmlRpc::XmlRpcValue& pos = pose["position"];
		XmlRpc::XmlRpcValue& ori = pose["orientation"];
  
		goal.pose.position.x = static_cast<double>(pos["x"]);
		goal.pose.position.y = static_cast<double>(pos["y"]);
		goal.pose.position.z = static_cast<double>(pos["z"]);
		
		goal.pose.orientation.x = static_cast<double>(ori["x"]);
		goal.pose.orientation.y = static_cast<double>(ori["y"]);
		goal.pose.orientation.z = static_cast<double>(ori["z"]);
		goal.pose.orientation.w = static_cast<double>(ori["w"]);
		ROS_WARN("x %f y %f z %f",goal.pose.position.x,goal.pose.position.y,goal.pose.position.z);
	  } 
	  catch (const XmlRpc::XmlRpcException& e) 
	  {
		ROS_WARN("Skipping invalid goal (index %zu): %s", index, e.getMessage().c_str());
	  }
	  index++;
	  return goal;
	}
	else
	{
		ROS_WARN("No more valid waypoints");
	}
	
}

bool goalReached(geometry_msgs::PoseStamped& msg, char z)
{
	if(!get_oringin_pos)//未收到里程计定位数据时返回false
	{
		return false;
	}	
	dx=local_pos.pose.position.x - msg.pose.position.x;
	dy=local_pos.pose.position.y - msg.pose.position.y;
	//ROS_WARN("x=%f,y=%f,z=%f",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
	if(z!=0)
	{
		dz=local_pos.pose.position.z - msg.pose.position.z;
	}
	else
	{
		dz=0;
	}
	if(sqrt(dx*dx + dy*dy + dz*dz) <= tolerance)
	{
		return true;
	}
	else
	{
		return false;
	}

}
bool FCUready()
{
    if(!current_state.connected)
    {
        ROS_WARN("FCU NOT CONECTED");
        return false;
    }
	//ROS_INFO("FCU CONECTED");
    if(current_state.mode != "OFFBOARD")
    {
        ROS_INFO("NOT IN OFFBOARD");
        return false;
    }
    if(!current_state.armed)
    {
        ROS_INFO("NOT ARMED");        
        return false;
    }
    else
    {
        return true;
    }
}
bool checkKeyPressedR() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    // 保存原有终端设置
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // 设置终端为无缓冲、不回显模式
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

    // 设置非阻塞模式
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();  // 尝试读取字符

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return (ch == 'r' || ch == 'R');  // 支持大小写
}
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    local_vel = *msg;
}
void cam_target_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
	cam_target = *msg;
	if(!cam_target_reached && takeoff_done )
	{
		NWTS.nav_waypoint_type_switch = NWTS.NAV_WYPT_CAM;
	}
	
}
int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "offb_NAV");
    ros::NodeHandle nh;
	ros::Rate rate(20.0);// 50ms
	ros::Time last_request = ros::Time::now();

	ros::Subscriber cam_target_sub = nh.subscribe<geometry_msgs::Vector3>
			("/aruco_horizontal_distance", 10, cam_target_cb);
    // 订阅无人机当前状态 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // 订阅无人机当前位置（反馈消息） 
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity", 10, local_vel_cb);
	// 创建目标点发布话题
	ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
			("/mavros/setpoint_raw/local",10);
    // 服务的客户端（设定无人机的模式、状态）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	////////////////************订阅rviz 2D_NAV_GOAL的目标点******************//////////////////////////////////////////
	ros::Subscriber rviz_goal_sub = nh.subscribe("/move_base_simple/goal", 1, rviz_goal_cb);
	////////////////************订阅rviz 2D_NAV_GOAL的目标点******************//////////////////////////////////////////

	///////////////************发布目标点到planner : /offb_target_pos/target_pos*************///////////////////////////
	ros::Publisher nav_goal_pb = nh.advertise<geometry_msgs::PoseStamped>
			("/offb_target_pos/target_pos", 2);
	///////////////************发布目标点到planner : /offb_target_pos/target_pos*************///////////////////////////

	///////////////************发布导航点类型 : /nav_waypoint/wypt_type*************///////////////////////////
	ros::Publisher nav_wypt_mode_pb = nh.advertise<msg::NAV_WYPT_MODE>
			("/nav_waypoint/wypt_type", 1);
	///////////////************发布导航点类型 : /nav_waypoint/wypt_type*************///////////////////////////

    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	

    while(ros::ok())
	{

		if(!takeoff_done && FCUready())//检测飞控状态，进入预设导航点模式
		{
			//NWTS.nav_waypoint_type_switch=NWTS.NAV_WYPT_PRESET;
			NWTS.nav_waypoint_type_switch = NWTS.NAV_WYPT_TAKEOFF;
		}
		if(checkKeyPressedR())
		{
			//next_poniit_flag=true;
			goalmaunlreach_flag=true;
		}
		if(goalmaunlreach_flag==true)
		{
			goalmanulreached=true;
			goalmaunlreach_flag=false;
		}
		switch (NWTS.nav_waypoint_type_switch)
		{
		case NWTS.NAV_WYPT_PRESET: 
			NWM.nav_mode = NWM.TRAJ_TRACK;
			nav_wypt_mode_pb.publish(NWM);//设置导航模式
			//if(!mission_start || goalReached(_WayPoints) )//|| goalReached(rviz_goal))

			if(!mission_start)
			{next_poniit_flag=true;mission_start=true;ros::Duration(5).sleep();}
			if(preset_flag==true && goalReached(_WayPoints,1))
			{next_poniit_flag=true;ROS_WARN("wypt_reached");}
			if(rviz_flag==true && goalReached(rviz_goal,0))
			{next_poniit_flag=true;ROS_WARN("rviz reached");}
/* 			if(preset_flag==true && goalmanulreached)
			{next_poniit_flag=true;ROS_WARN("wypt_reached");goalmanulreached=false;}
			if(rviz_flag==true && goalmanulreached)
			{next_poniit_flag=true;ROS_FATAL("rviz reached");goalmanulreached=false;} */
			if (next_poniit_flag==true)
			{	
				//getNextWaypoint();
				_WayPoints = getNextNavGoal();
				ROS_WARN("frame_id:%s",_WayPoints.header.frame_id.c_str());
				if(_WayPoints.header.frame_id !="invalid")
				{
					ROS_WARN("%d",_WayPoints.header.seq);
					nav_goal_pb.publish(_WayPoints);//发布预设导航点
					ROS_INFO("sent_num=%d",num_cnt_);
					num_cnt_++;
				}
				
				next_poniit_flag=false;
				preset_flag=true;
				rviz_flag=false;

			}
			break;
		case NWTS.NAV_WYPT_RVIZ:
			NWM.nav_mode = NWM.TRAJ_TRACK;
			nav_wypt_mode_pb.publish(NWM);
			nav_goal_pb.publish(rviz_goal);//发布RVIZ导航点
			ROS_ERROR("RVIZ");
			IS_RVIZ_GOAL = rviz_goal.header.seq + 1;//等待下次进入
			ROS_INFO("IS_RVIZ_=%d",IS_RVIZ_GOAL);
			NWTS.nav_waypoint_type_switch=NWTS.NAV_WYPT_PRESET;//回到预设导航点模式
			break;
		case NWTS.NAV_WYPT_CAM://摄像头目标
			NWM.nav_mode = NWM.CAM_TARGET;
			nav_wypt_mode_pb.publish(NWM);
			ROS_INFO("CAM.x=%f",cam_target.x);
			ROS_INFO("CAM.y=%f",cam_target.y);
			if(abs(cam_target.x)<=0.05 && abs(cam_target.y)<=0.05)
			{
				//NWTS.nav_waypoint_type_switch = NWTS.NAV_WYPT_PRESET;
				ROS_WARN("CAM_ATRGET REACHED");
				cam_target_reached=true;
				NWTS.nav_waypoint_type_switch = NWTS.NAV_WYPT_LAND;
			}
		break;
		case NWTS.NAV_WYPT_LAND:   // 降落
			ROS_WARN("LANDING");
			NWM.nav_mode = NWM.LAND;
			nav_wypt_mode_pb.publish(NWM);
			break;
		case NWTS.NAV_WYPT_TAKEOFF:   // 起飞
			NWM.nav_mode = NWM.TAKEOFF;
			nav_wypt_mode_pb.publish(NWM);
			ROS_INFO("Taking off");
			ROS_WARN("TAKEOFF_Z=%F",local_pos.pose.position.z);
			if(local_pos.pose.position.z>1.4 && local_pos.pose.position.z<1.6)
			{	ROS_INFO("TAKEOFF DONE!");
				takeoff_done=true;
				NWTS.nav_waypoint_type_switch=NWTS.NAV_WYPT_PRESET;
			}
			
		break;
		case NWTS.NAV_WYPT_RETURN://返回起飞点并降落
			NWM.nav_mode = NWM.RETURN;
			nav_wypt_mode_pb.publish(NWM);
			if(!goalReached(home_pos,0))
			{
				nav_goal_pb.publish(home_pos);
				ROS_WARN("RETURN");
			}
		break;
		default:
			break;
		}

        ros::spinOnce();
        rate.sleep();   // 影响消息发布与更新的周期
    }

    return 0;
}