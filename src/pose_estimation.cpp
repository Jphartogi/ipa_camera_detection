#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include "/home/jphartogi/catkin_ws/devel/include/hough_circle/ThresholdConfig.h"
#include <vector>

class PoseEstimation
{
private:
cv::Mat cameraMatrix;
cv::Mat distortionCoeffs;
bool haveCamInfo; std::string frameId;
//std::vector<cv::Vec3d>  rvecs, tvecs;

public:

PoseEstimation() {
    cameraMatrix = cv::Mat(3,3, cv::DataType<double>::type);
    distortionCoeffs = cv::Mat(4,1,cv::DataType<double>::type);
}

void Callback(const geometry_msgs::Vector3::Ptr &msg){

    double radius = msg->z;
    double x = msg->x;
    double y = msg->y;
    std::vector<cv::Point3f> origin; 
        
    origin.push_back(cv::Point3f(0,radius,0));
    origin.push_back(cv::Point3f(radius, 0 , 0));
    origin.push_back(cv::Point3f(0 , -radius , 0));
    origin.push_back(cv::Point3f(-radius, 0 , 0));

    std::vector<cv::Point2f> object;
    object.push_back(cv::Point2f(x + radius,y));
    object.push_back(cv::Point2f(x, y + radius));
    object.push_back(cv::Point2f(x - radius, y));
    object.push_back(cv::Point2f(x, y - radius));

    //std::vector<std::vector<double> > cameraMat = {{619.016845703125, 0.0, 331.33929443359375}, 
    //                                                {0.0, 618.93115234375, 226.82150268554688}, 
    //                                                {0.0, 0.0, 1.0}};
    cameraMatrix = 0;
    cameraMatrix.at<double>(0,0) = 619.016845703125;
    cameraMatrix.at<double>(1,1) = 618.93115234375;
    cameraMatrix.at<double>(2,2) = 1.0;
    cameraMatrix.at<double>(0,3) = 331.33929443359375;
    cameraMatrix.at<double>(1,3) = 226.82150268554688;

    
   // std::vector<double> t = {0.0, 0.0, 0.0, 0.0, 0.0};
    distortionCoeffs = 0;

    ROS_INFO("OK");
    
    cv::Mat rvecs(3,1,cv::DataType<double>::type);
    cv::Mat tvecs (3,1,cv::DataType<double>::type);
    cv::solvePnP(origin,object, cameraMatrix, distortionCoeffs,rvecs,tvecs);
                    

    // std::cout << rvecs[0] << ", " << rvecs[1] << ", " << rvecs[2] << std::endl;
}


};
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pose_circle");
	ros::NodeHandle nh;
    PoseEstimation pose;
    

	ros::Subscriber sub = nh.subscribe("/hough_circle/circlepos",10,&PoseEstimation::Callback,&pose);
    // ros::Subscriber cam_sub = nh.subscribe("/camera/color/camera_info", 10,
                    // &PoseEstimation::camInfoCallback, &pose);
    ros::spin();
    return 0;
}
