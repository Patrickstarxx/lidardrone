/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>

#include <quadrotor_msgs/PositionCommand.h>
#include <msg/NAV_WYPT_MODE.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define VELOCITY2D_CONTROL 0b101111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
bool allow_yaw = false;//如果不想要机头转动，则把这个设置为false，专门适配于雷达用的
bool have_cam = false;//检测摄像头信号
static ros::Time last_cam_time;
const double kp = 0.1;        // 比例系数
const double ki = 0.03;       // 积分系数
const double kd = 0.07;        // 微分系数
const double max_speed = 1.0; // 最大速度
float dt=0.02; //运行时间
double integral = 0.0;    // 积分项
double prev_error = 0.0;  // 上次误差
bool have_ego=false;

template<typename T>
T clamp(const T& value, const T& min_val, const T& max_val) {
    return (value < min_val) ? min_val : 
            ((value > max_val) ? max_val : value);
}

double pid_calculate(double current_error,double kp, double ki, double kd,double max_speed,double integral, double prev_error,double dt);
class Ctrl
{
    public:
        Ctrl();
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void position_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
        void control(const ros::TimerEvent&);
        bool FCUready(void);
        void nav_mode_cb(const msg::NAV_WYPT_MODE::ConstPtr& msg);
        void cam_target_cb(const geometry_msgs::Vector3::ConstPtr& msg);
	    ros::NodeHandle nh;
        visualization_msgs::Marker trackpoint;
        quadrotor_msgs::PositionCommand ego;
        msg::NAV_WYPT_MODE NAV_MODE;
        tf::StampedTransform ts;//用来发布无人机当前位置的坐标系坐标轴
        tf::TransformBroadcaster tfBroadcasterPointer;	//广播坐标轴
        unsigned short velocity_mask = VELOCITY2D_CONTROL;
        mavros_msgs::PositionTarget current_goal;
        mavros_msgs::RCIn rc;
        nav_msgs::Odometry position_msg;
        geometry_msgs::PoseStamped target_pos;
        //mavros_msgs::PositionTarget takeoff_pos;
        mavros_msgs::State current_state;
        geometry_msgs::Vector3 cam_target;
        float position_x, position_y, position_z, now_x, now_y, now_yaw, current_yaw, targetpos_x, targetpos_y;
        float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z, ego_a_x, ego_a_y, ego_a_z, ego_yaw, ego_yaw_rate; //EGO planner information has position velocity acceleration yaw yaw_dot
        bool receive, get_now_pos;//触发轨迹的条件判断
        bool have_odom = false;
        ros::Subscriber state_sub, twist_sub, target_sub, position_sub, nav_wypt_mode_sub, cam_target_sub;
        ros::Publisher local_pos_pub, pubMarker;
        ros::ServiceClient set_mode_client;
        ros::Timer timer;
        
        mavros_msgs::SetMode offb_set_mode;
        //mavros_msgs::State current_state;
};
Ctrl::Ctrl()
{
    timer = nh.createTimer(ros::Duration(0.02), &Ctrl::control, this);
    state_sub = nh.subscribe("/mavros/state", 10, &Ctrl::state_cb, this);
    position_sub=nh.subscribe("/mavros/local_position/odom", 10, &Ctrl::position_cb, this);
    target_sub = nh.subscribe("move_base_simple/goal", 10, &Ctrl::target_cb, this);
    twist_sub = nh.subscribe("/position_cmd", 10, &Ctrl::twist_cb, this);
    cam_target_sub = nh.subscribe<geometry_msgs::Vector3>("/camera_displacement", 1, &Ctrl::cam_target_cb, this);
    nav_wypt_mode_sub = nh.subscribe("/nav_waypoint/wypt_type", 1, &Ctrl::nav_mode_cb, this);
    local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pubMarker = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    get_now_pos = false;
    receive = true;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
}
void Ctrl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}
void Ctrl::cam_target_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
	cam_target = *msg;
    have_cam = true;
    //ROS_FATAL("cam_target_cb");
    last_cam_time = ros::Time::now();
}
//read vehicle odometry
void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
	tf2::Quaternion quat;
	tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_frame";
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tfBroadcasterPointer.sendTransform(ts);
	if (!get_now_pos) 
	{
		now_x = position_msg.pose.pose.position.x;
		now_y = position_msg.pose.pose.position.y;
		tf2::Quaternion quat;
		tf2::convert(msg->pose.pose.orientation, quat);
		now_yaw = yaw;
		get_now_pos = true;
	}
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
	current_yaw = yaw;
    have_odom = true;
}

void Ctrl::target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//读取rviz的航点
{
    receive = true;
    target_pos = *msg;
    targetpos_x = target_pos.pose.position.x;
    targetpos_y = target_pos.pose.position.y;
}

//读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
void Ctrl::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
	ego_vel_x = ego.velocity.x;
	ego_vel_y = ego.velocity.y;
	ego_vel_z = ego.velocity.z;
	ego_yaw = ego.yaw;
	ego_yaw_rate = ego.yaw_dot;
    have_ego=true;
    ROS_WARN("twist_cb");
}

void Ctrl::control(const ros::TimerEvent&)
{
    if(!have_odom)
    {
        std::cout<<"---------------no odom!!-------------"<<std::endl;
        return;
    }
    if(!FCUready())
    {
        std::cout<<"---------------FCU NOT READY!!-------------"<<std::endl;
        //NAV_MODE.nav_mode=NAV_MODE::NAV_MODE_EMPTY;
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_goal.header.stamp = ros::Time::now();
        current_goal.type_mask = velocity_mask;
        current_goal.velocity.x = 0;//空指令，保证飞控进入offboard
        current_goal.velocity.y = 0;
        current_goal.velocity.z = 0;
        current_goal.yaw = 0;
        ROS_INFO("发送空指令\n");
        local_pos_pub.publish(current_goal); 
        return;
    }
    //NAV_MODE.nav_mode= NAV_MODE.CAM_TARGET;//调试
    switch (NAV_MODE.nav_mode)
    {
        
        case NAV_MODE.HOLD:
            current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            current_goal.header.stamp = ros::Time::now();
            current_goal.type_mask = velocity_mask;
            current_goal.velocity.x = (now_x - position_x)*1;//now_x为初始位置;position_x为实时更新的当前位置
            current_goal.velocity.y = (now_y - position_y)*1;
            current_goal.velocity.z = (1.3 - position_z)*1;
            current_goal.yaw = now_yaw;
            ROS_INFO("无有效导航点，保持悬停\n");
        break;
        case NAV_MODE.TRAJ_TRACK:
            if(have_ego==true)
            {
                current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系，一定要local系
                current_goal.header.stamp = ros::Time::now();
                current_goal.type_mask = velocity_mask;//这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
                current_goal.velocity.x =  0.8 * ego_vel_x + (ego_pos_x - position_x)*0.6;
                current_goal.velocity.y =  0.8 * ego_vel_y + (ego_pos_y - position_y)*0.6;
                current_goal.velocity.z =  (ego_pos_z - position_z)*0.8;
                if(allow_yaw)
                    current_goal.yaw = ego_yaw;
                else
                    current_goal.yaw = now_yaw;
                ROS_INFO("已触发控制器，当前EGO规划速度：velocity = %.2f\n", sqrt(pow(current_goal.velocity.x, 2)+pow(current_goal.velocity.y, 2)));
            }
        break;
        case NAV_MODE.CAM_TARGET:
        {
            current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//选择local系，一定要local系
            current_goal.header.stamp = ros::Time::now();
            current_goal.type_mask = velocity_mask;//这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式

            if(!have_cam)
             {
                ROS_WARN("CAM LOST");
                current_goal.velocity.x = 0; 
                current_goal.velocity.y = 0; 
                current_goal.velocity.z = 0;
    
                current_goal.yaw = now_yaw;
            }
            else
            {
                ROS_INFO("摄像头目标");
                double x_error = 0 - cam_target.x;
                double y_error = 0 - cam_target.y;
                current_goal.velocity.x = pid_calculate(x_error, kp, ki, kd, max_speed, integral, prev_error, dt ); 
                current_goal.velocity.y = pid_calculate(y_error, kp, ki, kd, max_speed, integral, prev_error, dt ); 
                current_goal.velocity.z = 0;
    
                current_goal.yaw = now_yaw;
                //have_cam = false;
            }
        
            if ((ros::Time::now() - last_cam_time).toSec() > 0.1) 
            { // 0.1秒无数据视为丢失
                have_cam = false;
            }
            ROS_INFO("vel.x=%f",current_goal.velocity.x);
            ROS_INFO("vel.y=%f",current_goal.velocity.y);
            

        }
        break;
        case NAV_MODE.LAND://降落
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            ROS_INFO("LAND");
            if (current_state.mode != "AUTO.LAND" )// && (ros::Time::now() - last_request > ros::Duration(5.0))
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("AUTO.LAND enabled");
                }
                //last_request = ros::Time::now();
            }
        }
        break;
        case NAV_MODE.TAKEOFF:
        {
            ROS_INFO("起飞");
            current_goal.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX  |
            mavros_msgs::PositionTarget::IGNORE_VY  |
            mavros_msgs::PositionTarget::IGNORE_VZ  |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            current_goal.position.x=now_x;
            current_goal.position.y=now_y;
            current_goal.position.z=1.5;
            //local_pos_pub.publish(takeoff_pos);
        }
        break;
        default:
            break;
    }
    //ROS_FATAL("vel.x=%f",current_goal.velocity.x);
    //ROS_FATAL("vel.y=%f",current_goal.velocity.y);
    local_pos_pub.publish(current_goal); 

}
bool Ctrl::FCUready()
{
    if(!current_state.connected)
    {
        ROS_INFO("FCU NOT CONECTED");
        return false;
    }
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
double pid_calculate(double current_error,double kp, double ki, double kd,double max_speed,double integral,double prev_error,double dt)
{
// 计算微分项（带时间步长保护）
double derivative = (dt > 1e-6) ? 
       (current_error - prev_error) / dt : 0.0;

// 临时积分计算
double temp_integral =  integral + current_error * dt;

// 计算原始输出
double raw_output = kp * current_error + 
       ki * temp_integral + 
       kd * derivative;

// 输出限幅
double output = clamp(raw_output, -max_speed, max_speed);

// 抗积分饱和处理
if (std::abs(raw_output) <= max_speed) {
 integral = temp_integral;  // 仅当未饱和时保留积分
}

// 更新误差记录
 prev_error = current_error;

return output;
}
void Ctrl::nav_mode_cb(const msg::NAV_WYPT_MODE::ConstPtr& msg)
{
    NAV_MODE = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cxr_egoctrl_v1");
	setlocale(LC_ALL,"");
    Ctrl ctrl;
    ros::spin();
	return 0;
}