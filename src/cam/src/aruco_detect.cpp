#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>

// 全局变量
cv::Ptr<cv::aruco::Dictionary> dictionary;
cv::Ptr<cv::aruco::DetectorParameters> parameters;
cv::Mat cameraMatrix, distCoeffs;
Eigen::Vector3d drone_position;
Eigen::Quaterniond drone_orientation;

// 回调函数，用于从mavros获取无人机的位姿
void dronePoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    drone_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    drone_orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

// 回调函数，用于从USB摄像头获取图像
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("IMAGE");
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        // 转换为灰度图像
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 检测Aruco二维码
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        if (markerIds.size() > 0) {
            // 估计二维码的位姿
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.1, cameraMatrix, distCoeffs, rvecs, tvecs);

            // 计算二维码中心与摄像头中心的水平相对距离
            for (int i = 0; i < markerIds.size(); ++i) {
                Eigen::Vector3d marker_position(tvecs[i][0], tvecs[i][1], tvecs[i][2]);

                // 考虑无人机的高度和姿态
                Eigen::Matrix3d rotation_matrix = drone_orientation.toRotationMatrix();
                Eigen::Vector3d camera_position = drone_position + rotation_matrix * marker_position;

                double horizontal_distance = sqrt(camera_position.x() * camera_position.x() + camera_position.y() * camera_position.y());

                ROS_INFO("二维码中心与摄像头中心的水平相对距离: %.2f 米", horizontal_distance);
            }
        }

        // 显示图像
        cv::imshow("Aruco Detection", frame);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_landing");
    ros::NodeHandle nh;
	ROS_INFO("START");
    // 加载Aruco字典和参数
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters = cv::aruco::DetectorParameters::create();

    // 加载相机内参和畸变参数
    cameraMatrix = (cv::Mat_<double>(3, 3) << 406.932130, 0, 316.629381, 0, 402.678201, 242.533947, 0, 0, 1);
    distCoeffs = (cv::Mat_<double>(5, 1) << 0.039106, -0.056494, -0.000824, 0.092161, 0.0);

    // 订阅USB摄像头图像话题
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 10, imageCallback);

    // 订阅mavros的无人机位姿话题
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, dronePoseCallback);

    ros::spin();

    return 0;
}
