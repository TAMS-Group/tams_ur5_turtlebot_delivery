
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/StatePropagator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  moveit::planning_interface::MoveGroup group("manipulator");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(240);
  
  group.clearPoseTargets();
  group.clearPathConstraints();
  
  geometry_msgs::Pose pose_target;
  pose_target.orientation.x = 0.5;
  pose_target.orientation.y = 0.5;
  pose_target.orientation.z = -0.5;
  pose_target.orientation.w = 0.5;
  pose_target.position.x = 0.7;
  pose_target.position.y = 0.4;
  pose_target.position.z = 1.2;
  
  group.setPoseTarget(pose_target);


  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  success = group.execute(my_plan);
  ROS_INFO("Execution plan 1 (pose goal) %s",success?"":"FAILED");
  sleep(1.0);

  group.attachObject("flasche3");
  sleep(2.0);

  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation = pose_target.orientation;
  start_pose2.position.x = 0.7;
  start_pose2.position.y = 1.0;
  start_pose2.position.z = 0.6;


  group.setPoseTarget(start_pose2);
  success = group.plan(my_plan);
  sleep(10.0);
  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
  success = group.execute(my_plan);
  ROS_INFO("Execution plan 3 (constraints) %s",success?"":"FAILED");


  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();
  group.clearPoseTargets();
  
  group.detachObject("flasche3");
  sleep(2);

  ros::shutdown();  
  return 0;
}

