cmake_minimum_required(VERSION 2.8.3)
project(project15_objectrecognition)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	genmsg
	tf
	tf2
	tf2_sensor_msgs
	tf2_ros
	sensor_msgs
	pcl_conversions
	pcl_ros
	#pcl_msgs
	cv_bridge
	image_transport)
find_package(PCL REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)

catkin_package(
  LIBRARIES
	project15_objectrecognition
	pcl_ros_filters
	pcl_ros_io
	pcl_ros_tf
  CATKIN_DEPENDS 
	pcl_conversions
  DEPENDS 
	PCL
	opencv
)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})


add_executable(object_recognition src/object_recognition.cpp)
target_link_libraries(object_recognition ${catkin_LIBRARIES} ${PCL_LIBRARIES})
