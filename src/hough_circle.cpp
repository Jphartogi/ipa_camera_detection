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
#include <hough_circle/ThresholdConfig.h>
#include <visualization_msgs/Marker.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cstdlib>
#include <math.h>           
#include <cmath>
#include <vector>
#include <array>


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
    int average_y_points;
    std::vector< int > det_points;
    

    int center_x; int center_y; double radius;

    cv::Mat src, src_gray, src_blur , src_binary;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Mat objectpoints; cv::Mat circlepoints;
    image_transport::ImageTransport it;

    image_transport::Publisher pub;
    image_transport::Publisher bin_pub;
    image_transport::Subscriber sub;
    
    
    ros::Publisher vector_pub;
    ros::Publisher twist_publisher_;
    ros::Publisher pub_marker_;

    ros::Subscriber caminfo_sub;
    std::string frameId;

    sensor_msgs::ImagePtr msg;
    sensor_msgs::ImagePtr bin;

    bool haveCamInfo;
    bool allow_camera_rotation;
    bool rotate_camera;
    bool circle_detected;
    bool circle_is_valid;
        

    dynamic_reconfigure::Server<hough_circle::ThresholdConfig> server;
	dynamic_reconfigure::Server<hough_circle::ThresholdConfig>::CallbackType f;

    void houghDetection(const cv::Mat& src_blur, const cv::Mat& src_display,
                        int cannyThreshold, int accumulatorThreshold);

    void callback(hough_circle::ThresholdConfig &config, uint32_t level);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void run();
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void getMarker(sensor_msgs::CameraInfo camera_info);
    int positionAverage(std::vector<int> y);
    
    // void contourDetection(const cv::Mat& src_blur);
    
public:

    // Constructor
    Hough(ros::NodeHandle & nh) : it(nh)
    {
    // Camera intrinsics
    int circle_detected = 0;
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    haveCamInfo = false;
    allow_camera_rotation = false;
    rotate_camera = false;
    circle_detected = false;
    circle_is_valid = true;

    f = boost::bind(&Hough::callback,this, _1, _2);
	server.setCallback(f);

    pub = it.advertise("/houghcircle", 1);//Filtered image publisher
    sub = it.subscribe("/camera/color/image_raw", 1,
                    &Hough::imageCallback,this);
    bin_pub = it.advertise("/houghcircle_binary",1);


    vector_pub = nh.advertise<geometry_msgs::Vector3>("circlepos",10);
    pub_marker_ = nh.advertise<visualization_msgs::Marker>("cam_fov",10);

    twist_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    caminfo_sub = nh.subscribe("/camera/color/camera_info", 1,
                    &Hough::camInfoCallback, this);

    ROS_INFO("Hough circle detection ready, Initialization complete");
    }

    // Destructor
    ~Hough()
    {

    }

};

    
    void Hough::houghDetection(const cv::Mat& src_blur, const cv::Mat& src_display, int cannyThreshold, int accumulatorThreshold)
    {
        // will hold the results of the detection
        std::vector<cv::Vec3f> circles;
        
        // runs the actual detection
        HoughCircles(src_blur, circles, CV_HOUGH_GRADIENT, 2, 800, cannyThreshold, accumulatorThreshold, min_radius, max_radius );
        // print(circles);
            
            display = src_display.clone();
            
                                
                for( size_t i = 0; i < circles.size(); i++ )
                {
                    if (335 < circles[i][0] && circles[i][0] < 345 )
                    {   
        // clone the colour, input image for displaying purposes
                    cv::Point center(circles[i][0], circles[i][1]);
                    radius = circles[i][2];
               
                    // circle center
                    circle( display, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                    // circle outline
                    circle( display, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
                    
                   center_x = circles[i][0]; center_y = circles[i][1];  
                   circle_detected = true;
                   
                    if (detected_circle > 4) {
                    detected_circle = 0;
                   
                    }
                                   

                   if (detected_circle >= 0 && detected_circle < 1) {
                       det_points.push_back(center_y);
                       
                   }
                   else if (detected_circle >= 1 && detected_circle < 2) {
                       det_points.push_back(center_y);
                       positionAverage(det_points);
                       det_points.erase (det_points.begin(),det_points.end());
                   }
                                       
                    detected_circle = detected_circle + 1;
                                                
                    }
                else
                    {
                    //do nothing
                    ROS_WARN("that is the wrong circle detected!");
                    }

                    
                          
              }
              
              

    }   
    int Hough::positionAverage(std::vector<int> y)
    {
            for(size_t i = 0; i < y.size(); i++)
            {
                
                if (i > 0) {
                    if (abs(y[i]-y[i-1]) >= 15) {
                    ROS_INFO("detected circle is not correct, because there is jumping of circle!");
                    circle_is_valid == false;
                                        
                    }
                }
                if(circle_is_valid)
                {
                    if (y.size() > 0) {
                        average_y_points = (y[0] + y[1] )/2;
                        std::cout << "averagenya " << average_y_points << std::endl;
                        return average_y_points;
                        circle_is_valid == true;
                    }
                    
                    
                       
                }
                

            }
                
                
                 
    }
           
    

    void Hough::callback(hough_circle::ThresholdConfig &config, uint32_t level)
    {
    cannyThresholdInitialValue=config.canny_thres;
    accumulatorThresholdInitialValue= config.accumulator_thres;
    radius_real = config.radius;
    max_radius = config.max_radius;
    min_radius = config.min_radius;
    binary_thres = config.binary_thres;
    }

    void Hough::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // Read the image
	cv_bridge::CvImagePtr src_; //OpenCV full Image variable
	try
	    {
		src_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //Conversion
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
         cv::cvtColor( src, src_gray, cv::COLOR_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
        cv::GaussianBlur( src_gray, src_blur, cv::Size(5, 5), 2, 2 );
    // Use the binary threshold 
        
    //declare and initialize both parameters that are subjects to change
        int cannyThreshold = cannyThresholdInitialValue;
        int accumulatorThreshold = accumulatorThresholdInitialValue;
        
        //runs the detection, and update the display with messages
        houghDetection(src_blur, src, cannyThreshold, accumulatorThreshold);
        
        // contourDetection(src_blur);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", display).toImageMsg();
        
        
        pub.publish(msg);
        

        }

    }


    void Hough::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        
    sensor_msgs::CameraInfo camerainfo = *msg;
    getMarker(camerainfo);

    if (haveCamInfo) {
        return;
    }
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

    void Hough::getMarker(sensor_msgs::CameraInfo camerainfo)
    {
        
        bool show_marker;
        double min_distance, max_distance;
        
        image_geometry::PinholeCameraModel pinmodel;
        pinmodel.fromCameraInfo(camerainfo);

        if (center_x > 480 || center_y > 640) {
            // do nothing
            // because it's not possible, bigger than the maximum pixel
        }
        else
        {
            visualization_msgs::Marker cam_poly;	// create marker
            cam_poly.header = camerainfo.header;
            cam_poly.action = visualization_msgs::Marker::ADD;
            cam_poly.id = 0;
            cam_poly.ns = "fov";
            cam_poly.type = visualization_msgs::Marker::LINE_STRIP;	// ..as line strip
            cam_poly.scale.x = 0.01;
            cam_poly.color.g = 1.0; // set colors
            cam_poly.color.b = 1.0;
            cam_poly.color.a = 0.5; // set transparency


            // angle calculation

            // for example
            double base_to_circle = 0.167;
            double base_to_camera = 1;
            min_distance = 0.01; max_distance = sqrt(pow(base_to_circle,2)+pow(base_to_camera,2));
          
        // calc 3D Points out of boundaries of image. Result are four points  (rect at 1m distance)
            cv::Point3d P_topleft = pinmodel.projectPixelTo3dRay(cv::Point(0, 0));
            cv::Point3d P_downright = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width, camerainfo.height));
            cv::Point3d P_topright = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width, 0));
            cv::Point3d P_downleft = pinmodel.projectPixelTo3dRay(cv::Point(0, camerainfo.height));
            cv::Point3d P_center = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width/2, camerainfo.height/2));
            cv::Point3d P_circle = pinmodel.projectPixelTo3dRay(cv::Point(center_x, average_y_points));
           
            // project rect into desired distances (min_distance and max_distance)

            // vertices front rect
            geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8, p9 , p10, p_ref;
            p1.x = P_topleft.x * min_distance;
            p1.y = P_topleft.y * min_distance;
            p1.z = P_topleft.z * min_distance;

            p2.x = P_topright.x * min_distance;
            p2.y = P_topright.y * min_distance;
            p2.z = P_topright.z * min_distance;

            p3.x = P_downright.x * min_distance;
            p3.y = P_downright.y * min_distance;
            p3.z = P_downright.z * min_distance;

            p4.x = P_downleft.x * min_distance;
            p4.y = P_downleft.y * min_distance;
            p4.z = P_downleft.z * min_distance;

            // vertices rear rect
            p5.x = P_topleft.x * max_distance;
            p5.y = P_topleft.y * max_distance;
            p5.z = P_topleft.z * max_distance;

            p6.x = P_topright.x * max_distance;
            p6.y = P_topright.y * max_distance;
            p6.z = P_topright.z * max_distance;

            p7.x= P_downright.x * max_distance;
            p7.y= P_downright.y * max_distance;
            p7.z= P_downright.z * max_distance;

            p8.x= P_downleft.x * max_distance;
            p8.y= P_downleft.y * max_distance;
            p8.z= P_downleft.z * max_distance;
                        
            p10.x = P_center.x * max_distance;
            p10.y = P_circle.y * max_distance;
            p10.z = P_circle.z * max_distance;

            //set the starting point of the camera color (please calibrate(?))
            p9.x = P_center.x * min_distance;
            p9.y = P_center.y * min_distance;
            p9.z = P_center.z * min_distance;

            p_ref.x = P_center.x * max_distance;
            p_ref.y = P_center.y * max_distance;
            p_ref.z = sqrt(pow(base_to_circle,2)+pow(base_to_camera,2));

            // to make 45 degrees angle, base_circle and base_camera = 1:1
            // base_to_camera = 0.5, or base_to_circle = 1.4, the only possible way is base_to_camera = 0.5 with projection.
            

            // get reference angle for calculation the degree of camera
            double angle_ref = atan(base_to_circle/base_to_camera)*180.0/3.1415;;
            
            // get angle from detected circle line with the reference line
            double m_line = (p10.y - p9.y)/(max_distance - min_distance);
            double m_ref = (p_ref.y -p9.y)/(max_distance - min_distance);
            double angle;
            double x = std::abs((m_line-m_ref)/(1.0+(m_line*m_ref)));
            angle = atan(x)*180.0/3.1415; // in degrees
            // std::cout << "the angle detected is" << angle << std::endl;
            // std::cout << " and the reference angle is " << angle_ref << std::endl;
            // std::cout << " m line " << m_line << " mrefnya " << m_ref << std::endl;
            // std::cout << " the x is" << x << std::endl;
            // std::cout << "the degree is  " << angle << std::endl;
         
            if (p10.y > p_ref.y) {
                angle = angle + angle_ref;
            }
            else
            {
                angle = std::abs(angle - angle_ref);
            }
            // std::cout << "the degree is  " << angle << std::endl;
           
            
            // push back points to get line polynom
            cam_poly.points.push_back(p1); cam_poly.points.push_back(p2); cam_poly.points.push_back(p3);
            cam_poly.points.push_back(p4); cam_poly.points.push_back(p1); cam_poly.points.push_back(p5);
            cam_poly.points.push_back(p6); cam_poly.points.push_back(p7); cam_poly.points.push_back(p8);
            cam_poly.points.push_back(p5); cam_poly.points.push_back(p8); cam_poly.points.push_back(p4);
            cam_poly.points.push_back(p3); cam_poly.points.push_back(p7); cam_poly.points.push_back(p6);
            cam_poly.points.push_back(p2); cam_poly.points.push_back(p9); cam_poly.points.push_back(p10);
            cam_poly.points.push_back(p9); cam_poly.points.push_back(p_ref);
            
            pub_marker_.publish(cam_poly);


        }
       
    }

    
    
    
int main(int argc, char** argv)
{
    ros::init(argc,argv,"hough_circle");
    ros::NodeHandle nh("~");
    //Hough * node = new Hough(nh);
    Hough Hough(nh);

    ros::Rate rate(5);
    ros::spin();

    return 0;

}
