source ./devel/setup.bash
roslaunch mavros px4.launch & sleep 10;
roslaunch lidar_to_mavros lidar_to_mavros.launch & sleep 5;
rosrun mavros mavcmd long 511 32 8000 0 0 0 0 0;
roslaunch ego_planner single_run_in_exp.launch & sleep 5;
roslaunch controller wypts_ctrl.launch & sleep 5;
rosrun controller cxr_egoctrl_v1 & sleep 5;
rosrun cam aruco_img_multi.py;
roslaunch rosboard rosboard.launch
wait;
