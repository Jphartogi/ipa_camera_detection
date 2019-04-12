# Hough_circle
Circle detection with ROS and C++ code. Also adding some TF publisher

# Introduction
This package is used to detect a circle which uses Hough Circle Detection Function from OpenCV. More details about the hough circle 
can be read in https://docs.opencv.org/2.4.13.7/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html and the ROS version
http://wiki.ros.org/opencv_apps.

This package is specifically used to determine camera angle from detecting a circle.

# How to launch
```bash
 roslaunch hough_circle circle_detection.launch
```
You can also determine parameter such as (threshold,circle to circle distance in dynamic reconfigure)
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

# Author 

Joshua Phartogi https://github.com/Jphartogi

