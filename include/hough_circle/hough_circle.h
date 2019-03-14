
#define PI 3.14159265
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <hough_circle/ThresholdConfig.h>
#include <cstdlib>
#include <math.h>           
#include <cmath>
#include <vector>
#include <array>
#include "std_srvs/SetBool.h"

namespace ipa_hough_circle
{
class Hough
{

private:
    cv::Mat display;
    cv::Mat rvecs;
    cv::Mat tvecs;
        
    // initial and max values of the parameters of interests.

    int cannyThresholdInitialValue;
    int accumulatorThresholdInitialValue;
    int binary_thres;
    int max_radius;
    int min_radius;
    int detected_circle;
    double radius_real;
    double average_y_points;
    
    std::vector< double > det_points;
    
    ros::NodeHandle nh;

    int center_x; int center_y; double radius;

    cv::Mat src, src_gray, src_blur , src_binary;
    cv::Mat cameraMatrix ,distortionCoeffs , objectpoints , circlepoints;
    image_transport::ImageTransport it;

    image_transport::Publisher pub;
    image_transport::Publisher bin_pub;
    image_transport::Subscriber sub;
        
    ros::Publisher vector_pub;
    ros::Publisher pub_marker_;
    ros::Publisher pose_pub;

    ros::Subscriber caminfo_sub;
    std::string frameId;

    ros::ServiceServer service;

    sensor_msgs::ImagePtr msg;
    sensor_msgs::ImagePtr bin;

    bool haveCamInfo;
    bool allow_camera_rotation;
    bool rotate_camera;
    bool circle_detected;
    bool circle_is_valid;
    bool do_circle_detection;
    bool debug_mode;
    bool image_stream;
        
    dynamic_reconfigure::Server<hough_circle::ThresholdConfig> server;
	dynamic_reconfigure::Server<hough_circle::ThresholdConfig>::CallbackType f;

    void houghDetection(const cv::Mat& src_blur, const cv::Mat& src_display,
                        int cannyThreshold, int accumulatorThreshold);

    void callback(hough_circle::ThresholdConfig &config, uint32_t level);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void run();
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void getMarker(sensor_msgs::CameraInfo camera_info);
    double positionAverage(std::vector<double> y);
    //pose estimation
    void getPose(double angle,sensor_msgs::CameraInfo camerainfo);
    
    bool srv_cam_pose(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res );
     
    
public:
Hough();
};
}