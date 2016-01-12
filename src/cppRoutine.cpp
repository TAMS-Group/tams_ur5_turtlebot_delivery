
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


  moveit::planning_interface::MoveGroup group("UR5_arm");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(30);
  
  group.clearPoseTargets();
  group.clearPathConstraints();
  
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Plan 1
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  geometry_msgs::Pose pose_target;
  pose_target.orientation.x = 0.5;
  pose_target.orientation.y = 0.5;
  pose_target.orientation.z = -0.5;
  pose_target.orientation.w = 0.5;
  pose_target.position.x = 0.7;
  pose_target.position.y = 0.4;
  pose_target.position.z = 1.2;
  
  group.setPoseTarget(pose_target);
  group.move();

  //moveit::planning_interface::MoveGroup::Plan my_plan;
  //bool success = group.plan(my_plan);

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  //success = group.execute(my_plan);
  //ROS_INFO("Execution plan 1 (pose goal) %s",success?"":"FAILED");
  sleep(1.0);

  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Attach object
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame(); //"/world";

  /* The id of the object is used to identify it. */
  collision_object.id = "box1";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.07;
  primitive.dimensions[1] = 0.07;
  primitive.dimensions[2] = 0.28;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.7;
  box_pose.position.y =  0.4;
  box_pose.position.z =  0.9;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

  ROS_INFO("Attach the object to the robot");

  group.attachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(4.0);


  //robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose pose_target2;
  pose_target2.orientation = pose_target.orientation;
  pose_target2.position.x = 0.7;
  pose_target2.position.y = 1.0;
  pose_target2.position.z = 0.6;
  //group.setStartState(start_state);


  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Plan 2
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  group.setPoseTarget(pose_target2);
  group.move();
  //success = group.plan(my_plan);
  sleep(10.0);
  //ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
  //success = group.execute(my_plan);
  //ROS_INFO("Execution plan 3 (constraints) %s",success?"":"FAILED");

  ROS_INFO("Detach the object from the robot");
  group.detachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object detached. */
  sleep(4.0);


  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();
  group.clearPoseTargets();


  ros::shutdown();  
  return 0;
}

