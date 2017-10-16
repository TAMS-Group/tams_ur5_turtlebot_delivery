cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_turtlebot_delivery_turtlebot)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
   FILES
   itemRequest.msg
   dockingBridgeReply.msg
   dockingBridge.msg
   OnlineTurtle.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/mapPosition.py scripts/turtleAnnounce.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
