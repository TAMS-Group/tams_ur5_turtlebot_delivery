cmake_minimum_required(VERSION 2.8.3)
project(project15_manipulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  actionlib
  geometric_shapes
  moveit_ros_planning_interface
  tf
  project15_coordination
  robotiq_s_model_control
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package()

include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(cppActionServerPlaceObject src/cppActionServerPlaceObject.cpp)
target_link_libraries(cppActionServerPlaceObject ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(
  cppActionServerPlaceObject
  ${project15_coordination_EXPORTED_TARGETS}
)
install(TARGETS cppActionServerPlaceObject DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cppActionClientPlaceObject src/cppActionClientPlaceObject.cpp)
target_link_libraries(cppActionClientPlaceObject ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(
  cppActionClientPlaceObject
  ${project15_coordination_EXPORTED_TARGETS}
)
install(TARGETS cppActionServerPlaceObject DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
