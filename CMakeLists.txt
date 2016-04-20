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
find_package(OpenCV REQUIRED)

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

add_executable(camera_position_node src/camera_position_node.cpp)
target_link_libraries(camera_position_node ${catkin_LIBRARIES})

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(image_converter_MODE src/image_converter_MODE.cpp)
target_link_libraries(image_converter_MODE ${catkin_LIBRARIES} ${PCL_LIBRARIES})
