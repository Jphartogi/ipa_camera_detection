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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <hough_circle/ThresholdConfig.h>
#include <vector>


namespace enc = sensor_msgs::image_encodings;

using namespace cv;
//defining global variable (temporary)
class Hough
{
   
private:
    Mat display;
    // initial and max values of the parameters of interests.
    
    int cannyThresholdInitialValue = 200;
    int accumulatorThresholdInitialValue = 120;
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;
    cv::Mat src, src_gray;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Mat objectpoints; cv::Mat circlepoints;
    image_transport::ImageTransport it;

    image_transport::Publisher pub;
    image_transport::Subscriber sub;
    ros::Publisher vector_pub;

    ros::Subscriber caminfo_sub;
    std::string frameId;

    sensor_msgs::ImagePtr msg;

    bool haveCamInfo;

    dynamic_reconfigure::Server<hough_circle::ThresholdConfig> server;
	dynamic_reconfigure::Server<hough_circle::ThresholdConfig>::CallbackType f;

    void print(std::vector<Vec3f> const &input);
    void HoughDetection(const Mat& src_gray, const Mat& src_display,
                        int cannyThreshold, int accumulatorThreshold);
    
    void callback(hough_circle::ThresholdConfig &config, uint32_t level);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void run();
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void getPose(double radius, double x,double y);
    
public:
    
    Hough(ros::NodeHandle & nh) : it(nh)
    {
    cameraMatrix = cv::Mat(3,3, cv::DataType<double>::type);
    distortionCoeffs = cv::Mat(4,1,cv::DataType<double>::type);
    haveCamInfo = false;

    f = boost::bind(&Hough::callback,this, _1, _2);
	server.setCallback(f);

    pub = it.advertise("/houghcircle", 1);//Filtered image publisher
    sub = it.subscribe("/camera/color/image_raw", 1,
                    &Hough::imageCallback,this);
    vector_pub = nh.advertise<geometry_msgs::Vector3>("circlepos",10);

    caminfo_sub = nh.subscribe("/camera/color/camera_info", 1,
                    &Hough::camInfoCallback, this);

                    ROS_INFO("Hough circle detection ready");
    }
    
};
    void Hough::print(std::vector<Vec3f> const &input)
    {
	for (int i = 0; i < input.size(); i++) {
		std::cout << input.at(i) << ' ' << std::endl;
	    }
    }
    
     void Hough::HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold)
    {
        // will hold the results of the detection
        std::vector<Vec3f> circles; 

        // runs the actual detection
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 2, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );
        // print(circles);
        
                display = src_display.clone();
                std::cout << " the size of circles" << circles.size() << std::endl;
                for( size_t i = 0; i < circles.size(); i++ )
                {   
                   
        // clone the colour, input image for displaying purposes        
                    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                    int radius = cvRound(circles[i][2]);
                     if (radius >= 100) {
                    //do nothing
                    // to filter a bigger circle that is not needed
                    }
                    else
                    {      
                    Point firstpoint(cvRound(circles[i][0] + radius), cvRound(circles[i][1])); 
                    Point secondpoint(cvRound(circles[i][0]), cvRound(circles[i][1]) +radius); 
                    Point thirdpoint(cvRound(circles[i][0]) - radius, cvRound(circles[i][1])); 
                    Point fourthpoint(cvRound(circles[i][0]), cvRound(circles[i][1]) - radius);
                    // circle center
                    circle( display, center, 3, Scalar(0,0,255), -1, 8, 0 );
                    // circle outline
                    circle( display, center, radius, Scalar(0,255,0), 3, 8, 0 );
                    circle( display, firstpoint,3,Scalar(255,0,0), -1, 8, 0 );
                    circle( display, secondpoint,3,Scalar(255,0,0), -1, 8, 0 );
                    circle( display, thirdpoint,3,Scalar(255,0,0), -1, 8, 0 );
                    circle( display, fourthpoint,3,Scalar(255,0,0), -1, 8, 0 );
                   
                    // geometry_msgs::Vector3 vector;
                    // vector.x = cvRound(circles[i][0]); vector.y = cvRound(circles[i][1]); 
                    // vector.z = radius;
                    //  vector_pub.publish(vector);
                    getPose(radius,cvRound(circles[i][0]), cvRound(circles[i][1])); 
                            
                                    
                    }
            }
    }  

    void Hough::callback(hough_circle::ThresholdConfig &config, uint32_t level)
    {
    cannyThresholdInitialValue=config.canny_thres;
    accumulatorThresholdInitialValue= config.accumulator_thres;

    }

    void Hough::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {	  
        // Read the image
	cv_bridge::CvImagePtr src_; //OpenCV full Image variable
	try
	    {
		src_ = cv_bridge::toCvCopy(msg, enc::BGR8); //Conversion
		src = src_->image;//Assign just the image information to a Mat variable
	    }
	catch (cv_bridge::Exception& e)
	    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	    }
         int key = 0;
         run();
    }

    void Hough::run(){
        if(!src.empty())
        {
         // Convert it to gray
         cvtColor( src, src_gray, COLOR_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
          GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

    //declare and initialize both parameters that are subjects to change
        int cannyThreshold = cannyThresholdInitialValue;
        int accumulatorThreshold = accumulatorThresholdInitialValue;

        cannyThreshold = std::max(cannyThreshold, 1);
        accumulatorThreshold = std::max(accumulatorThreshold, 1);

        //runs the detection, and update the display with messages
        HoughDetection(src_gray, src, cannyThreshold, accumulatorThreshold);
        
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
                    
        pub.publish(msg);
        
	    //Filtered image publication
        // get user key
        
        }

    }

    void Hough::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) 
    {
       
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->K[i*3+j];
                
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->D[i];
        }
        
        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
         }
      
    }

    void Hough::getPose(double radius,double x,double y){

    cv::Mat rvecs(3,1,cv::DataType<double>::type);
    cv::Mat tvecs(3,1,cv::DataType<double>::type);
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

    cv::solvePnP(origin,object, cameraMatrix, distortionCoeffs,rvecs,tvecs);

    ROS_INFO("OK!");
    std::cout << rvecs << std::endl;
    // std::cout << " the camera matrix are " << cameraMatrix << std::endl;
    std::cout << "the radius are : " << radius << std::endl;

    double transform_x = tvecs.at<double>(0,0)/100.0;
    double transform_y = tvecs.at<double>(1,0)/100.0;
    double transform_z = tvecs.at<double>(2,0)/1000.0;

    double roll = rvecs.at<double>(0,0);
    double pitch = rvecs.at<double>(1,0);
    double yaw = rvecs.at<double>(2,0);     

    // double yaw = 0.7;
    // ROS_INFO("x = %f , y = %f, z = %f",transform_x,transform_y,transform_z);

    if (radius <= 20 || radius >= 100)
		{
			ROS_INFO("kesini dia boy");
			static tf::TransformBroadcaster br;
			tf::Transform trf2;
			trf2.setOrigin( tf::Vector3(0,0,0));
			trf2.setRotation(tf::Quaternion(0,0,0,1));   // set transform to 0 but not publishing!
			// br.sendTransform(tf::StampedTransform(trf2,ros::Time::now(),"camera_color_optical_frame","circle"));

		}
	else{
		/// when the circle is detected

		static tf::TransformBroadcaster br;
        tf::Transform transform_circle;
        transform_circle.setOrigin(tf::Vector3(transform_x,transform_y, transform_z));
        tf::Quaternion w;
        w.setRPY(roll,pitch,yaw);  // set the 2DOF orientation in Roll Pitch and Yaw. The orientation needed is only the yaw.
        transform_circle.setRotation(w);
            // transform_base_camera.setRotation(tf::Quaternion(rotation_x,rotation_y,rotation_z,rotation_w));
        br.sendTransform(tf::StampedTransform(transform_circle,ros::Time::now(),
        "camera_color_optical_frame","circle"));
        ros::Rate rate(1000);
                // rate.sleep();
	 
	    }
    }    

int main(int argc, char** argv)
{
    ros::init(argc,argv,"hough_circle");
    ros::NodeHandle nh("~");
    Hough * node = new Hough(nh);
    
    ros::Rate rate(5);
    ros::spin();
    
    return 0;
        
}
