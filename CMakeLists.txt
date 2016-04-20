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

catkin_package(
)

include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(planning_scene_setup src/planning_scene_setup.cpp)

target_link_libraries(planning_scene_setup
   ${catkin_LIBRARIES}
 )

add_executable(movegroup_plan1 src/movegroup_plan1.cpp)
target_link_libraries(movegroup_plan1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS movegroup_plan1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(movegroup_plan_pos1_pos2 src/movegroup_plan_pos1_pos2.cpp)
target_link_libraries(movegroup_plan_pos1_pos2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS movegroup_plan_pos1_pos2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cppGreifeTisch_legeBoden src/cppGreifeTisch_legeBoden.cpp)
target_link_libraries(cppGreifeTisch_legeBoden ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS cppGreifeTisch_legeBoden DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cppActionServerPlaceObject src/cppActionServerPlaceObject.cpp)
target_link_libraries(cppActionServerPlaceObject ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(
  cppActionServerPlaceObject
  ${project15_coordination}_EXPORTED_TARGETS
)
install(TARGETS cppActionServerPlaceObject DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cppActionClientPlaceObject src/cppActionClientPlaceObject.cpp)
target_link_libraries(cppActionClientPlaceObject ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(
  cppActionClientPlaceObject
  ${project15_coordination}_EXPORTED_TARGETS
)
install(TARGETS cppActionServerPlaceObject DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cppRoutine src/cppRoutine.cpp)
target_link_libraries(cppRoutine ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS cppRoutine DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cppRoutineUpdate src/cppRoutineUpdate.cpp)
target_link_libraries(cppRoutineUpdate ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS cppRoutineUpdate DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cppRoutineUpdate2 src/cppRoutineUpdate2.cpp)
target_link_libraries(cppRoutineUpdate2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS cppRoutineUpdate2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
