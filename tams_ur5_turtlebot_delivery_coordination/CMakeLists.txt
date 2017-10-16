cmake_minimum_required(VERSION 2.8.3)
project(project15_coordination)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  actionlib_msgs
  geometry_msgs
)

add_message_files(
  FILES
  task.msg
  taskStatus.msg
)

add_action_files(
  DIRECTORY action
  FILES PlaceObject.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/coordinator.py scripts/task.py scripts/turtle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
