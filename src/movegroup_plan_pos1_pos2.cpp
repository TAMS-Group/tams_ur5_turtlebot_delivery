
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

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  group.setPlannerId("SBLkConfigDefault");

  group.clearPoseTargets();
  group.clearPathConstraints();

  geometry_msgs::Pose pose_target;
  pose_target.orientation.x = 0.7071067811865476;
  pose_target.orientation.y = 0;
  pose_target.orientation.z = -0.7071067811865476;
  pose_target.orientation.w = 0;
  pose_target.position.x = 0.77;
  pose_target.position.y = 0.33;
  pose_target.position.z = 1.0;
  group.setPoseTarget(pose_target);


  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  success = group.execute(my_plan);
  ROS_INFO("Execution plan 1 (pose goal) %s",success?"":"FAILED");



  pose_target.orientation.x = 0.7071067811865476;
  pose_target.orientation.y = 0;
  pose_target.orientation.z = -0.7071067811865476;
  pose_target.orientation.w = 0;
  pose_target.position.x = 0.4;
  pose_target.position.y = 1.2;
  pose_target.position.z = 0.6;

  group.setPoseTarget(pose_target);

  
  moveit_msgs::OrientationConstraint ocm;  
  ocm.link_name = "ee_link";
  ocm.header.frame_id = "ee_link";
  ocm.orientation = pose_target.orientation;
  ocm.absolute_x_axis_tolerance = 3.1;
  ocm.absolute_y_axis_tolerance = 3.1;
  ocm.absolute_z_axis_tolerance = 3.1;
  ocm.weight = 1.0;
  
  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(constraints);

  


  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation = pose_target.orientation;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);



  group.setPoseTarget(pose_target);
  success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
  success = group.execute(my_plan);
  ROS_INFO("Execution plan 3 (constraints) %s",success?"":"FAILED");


  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();
  group.clearPoseTargets();
  

  ros::shutdown();  
  return 0;
}

