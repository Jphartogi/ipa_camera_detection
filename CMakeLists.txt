cmake_minimum_required(VERSION 2.8.3)
project(hough_circle)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  image_transport
  geometry_msgs
  visualization_msgs
  sensor_msgs
  image_geometry
  
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Threshold.cfg
#   cfg/DynReconf2.cfg
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hough_circle
  CATKIN_DEPENDS 
  cv_bridge dynamic_reconfigure roscpp rospy std_msgs image_transport geometry_msgs visualization_msgs sensor_msgs image_geometry
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

 add_executable(hough_circle src/hough_circle.cpp)




## Specify libraries to link a library or executable target against
 target_link_libraries(hough_circle
   ${catkin_LIBRARIES}
 )

target_link_libraries(hough_circle
   ${OpenCV_LIBRARIES}
 )


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 #install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
 #  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 #  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 #  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 #)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


