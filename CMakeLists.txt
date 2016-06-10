cmake_minimum_required(VERSION 2.8.3)
project(project15_objectrecognition)

find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
	pcl_ros
	roscpp
	tf
	visualization_msgs)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
	pcl_conversions
	visualization_msgs
  DEPENDS
	PCL
)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(object_recognition src/object_recognition.cpp)
target_link_libraries(object_recognition ${catkin_LIBRARIES} ${PCL_LIBRARIES})
