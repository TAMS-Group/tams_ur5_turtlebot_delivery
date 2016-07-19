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

add_executable(PlaceObjectActionServer src/PlaceObjectActionServer.cpp)
target_link_libraries(PlaceObjectActionServer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(
  PlaceObjectActionServer
  ${project15_coordination_EXPORTED_TARGETS}
)

add_executable(PlaceObjectActionClient src/PlaceObjectActionClient.cpp)
target_link_libraries(PlaceObjectActionClient ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(
  PlaceObjectActionClient
  ${project15_coordination_EXPORTED_TARGETS}
)

install(TARGETS PlaceObjectActionServer DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS PlaceObjectActionClient DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
