/**
 *         功能包：  roscpp std_msgs geometry_msgs mavros_msgs
 *         程序实现内容 ：使无人机飞行一个矩形轨迹后降落 （路径点）
 *          local_pos_pub.publish(pose);                 ——  2021/12/1    Poao
 * 
 *            话题/服务名称                                     操作                                                            消息类型															头文件
 *          mavros/cmd/arming                      服务的客户端（进入待机模式）    mavros_msgs::CommandBool			#include <mavros_msgs/CommandBool.h>	
 *          mavros/set_mode                          服务的客户端（设定工作模式）     mavros_msgs/Setmode					#include <mavros_msgs/SetMode.h>
 *          mavros/state                                 订阅 无人机状态                             mavros_msgs::State			              #include<mavros_msgs/State.h>		
 *          mavros/local_position/pose         订阅 无人位置消息                              geometry_msgs::PoseStamped		 #include <geometry_msgs/PoseStamped.h>
*           mavros/setpoint_position/local    发布 无人机控制位置                     geometry_msgs::PoseStamped         #include <geometry_msgs/PoseStamped.h>
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>

//订阅无人机状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
}

int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "offb_cfx");
    ros::NodeHandle nh;
    // 订阅无人机当前状态 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // 订阅无人机当前位置（反馈消息） 
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    // 发布无人机本地位置（控制）
/*     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10); */
	// 创建目标点发布话题
	ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
			("/mavros/setpoint_raw/local",10);

    // 服务的客户端（设定无人机的模式、状态）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");





		// 限制最大水平速度
		ros::ServiceClient param_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
		
		// 设置水平速度最大值
		mavros_msgs::ParamSet param_set;
		param_set.request.param_id = "MPC_XY_VEL_MAX";
		param_set.request.value.real = 1.0; // 1.0 m/s
		if (param_client.call(param_set)) {
			ROS_INFO("Set MPC_XY_VEL_MAX to 1.0 m/s: %s", param_set.response.success ? "Success" : "Fail");
		} else {
			ROS_ERROR("Failed to set MPC_XY_VEL_MAX");
		}








    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);// 50ms

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();//循环频率20hz(50ms)
		ROS_INFO("Waiting for FCU");
    }

	mavros_msgs::PositionTarget target;
	target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 使用局部NED坐标系
	target.type_mask = 0;

	target.position.x = 0;
	target.position.y = 0;
	target.position.z = 0;

	target.velocity.x = 0;
	target.velocity.y = 0;
	//target.velocity.z = 2;

	target.type_mask =
		mavros_msgs::PositionTarget::IGNORE_VZ  |
		mavros_msgs::PositionTarget::IGNORE_AFX |
		mavros_msgs::PositionTarget::IGNORE_AFY |
		mavros_msgs::PositionTarget::IGNORE_AFZ |
		mavros_msgs::PositionTarget::IGNORE_YAW |
		mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

   // geometry_msgs::PoseStamped pose;
/*     pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0; */

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        //local_pos_pub.publish(pose);
		setpoint_pub.publish(target);
        ros::spinOnce();
        rate.sleep();
    }

 // 设定无人机工作模式 offboard
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
// 无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

// 记录当前时间
    ros::Time last_request = ros::Time::now();

//  用于走圈的变量
	int step = 0;
	int sametimes = 0;

    while(ros::ok()){
        // 无人机状态设定与判断      
        // 进入while循环后，先循环5s，然后再向客户端发送无人机状态设置的消息
        // set_mode_client.call   arming_client.call 

                     //  无人机 Offboard enabled && Vehicle armed 后
                     //  无人机走矩形 每到达一个点停一会
                     //  z: 0-->10 10-->10 10-->10 10-->10  10-->10 10-->0 
                     //  x: 0-->0   0-->40   40-->40 40-->0    0-->0     0-->0
                     //  y: 0-->0   0-->0     0-->20   20-->20  20-->0   0-->0
                     //  local_pos_pub.publish(pose);
				switch (step)
				{
				case 0://模式offboard
					if( current_state.mode != "OFFBOARD" )
					{
						ROS_INFO("wait for offb_mode");
						if( current_state.mode == "OFFBOARD" )
						{
							ROS_INFO("Offboard enabled");
							step=1;
						}
						//last_request = ros::Time::now();
					} 
				case 1://解锁
					if( !current_state.armed )
					{
						ROS_INFO("Waiting for ARM");
						if(current_state.armed)
						{
							ROS_INFO("Vehicle armed");
							step=2;
						}
						//last_request = ros::Time::now();
					}

				case 2: 
					target.position.x = 0;
					target.position.y = 0;
					target.position.z = 1.5;
					ROS_INFO("Take off");
					//
					if (local_pos.pose.position.z > 1.4 && local_pos.pose.position.z < 1.6)
					{
						ROS_INFO("takeoff done!");
						if (sametimes > 20)
						{
							
							sametimes = 0;
							step = 3;

							target.position.x = 2;
							target.position.y = 0;
							target.position.z = 1.5;
							ROS_INFO("POS_1");
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}
					//local_pos_pub.publish(pose);
					setpoint_pub.publish(target);
					break;
				case 3:
					
					if (local_pos.pose.position.x > 1.9 && local_pos.pose.position.x < 2.1)
					{
						if (sametimes > 20)
						{
							
							step = 4;

							target.position.x = 2;
							target.position.y = 2;
							target.position.z = 1.5;
							ROS_INFO("POS_2");
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}

					setpoint_pub.publish(target);
					break;
				case 4:
					
					if (local_pos.pose.position.y > 1.9 && local_pos.pose.position.y < 2.1)
					{
						if (sametimes > 20)
						{
							
							step = 5;

							target.position.x = 0;
							target.position.y = 2;
							target.position.z = 1.5;
							ROS_INFO("POS_3");
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}

					setpoint_pub.publish(target);
					break;
				case 5:
					
					if (local_pos.pose.position.x > -0.1 && local_pos.pose.position.x < 0.1)
					{
						if (sametimes > 20)
						{
							
							step = 6;

							target.position.x = 0;
							target.position.y = 0;
							target.position.z = 1.5;
							ROS_INFO("POS_home");
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}

					setpoint_pub.publish(target);
					break;
				case 6:
					
					if (local_pos.pose.position.y > -0.1 && local_pos.pose.position.y < 0.1)
					{
						if (sametimes > 20)
						{
							step = 7;
						}
						else
							sametimes++;
					}
					else
					{
						sametimes = 0;
					}

					setpoint_pub.publish(target);
					break;
				case 7:   // 准备降落
					offb_set_mode.request.custom_mode = "AUTO.LAND";
					if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
					{

						if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
						{
							ROS_INFO("AUTO.LAND enabled");
						}
						last_request = ros::Time::now();
					}
					break;
				default:
					break;
				}

        // 发布位置控制信息
		setpoint_pub.publish(target);

        ros::spinOnce();
        rate.sleep();   // 影响消息发布与更新的周期
    }

    return 0;
}