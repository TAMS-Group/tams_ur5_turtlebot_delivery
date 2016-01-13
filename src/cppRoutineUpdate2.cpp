
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/StatePropagator.h>

#include <ros/console.h>
#include <stdio.h>




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
  ROS_INFO("Plan 1: Set start position");

  geometry_msgs::Pose pose_target1;
  pose_target1.position.x = 0.8;
  pose_target1.position.y = 0.8;
  pose_target1.position.z = 1.25;
  
  group.setPoseTarget(pose_target1);
  ROS_INFO("Move end effector to target position");
  group.move();

  sleep(1.0);


  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Load object
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame(); //"/world";

  //Bring up Object

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
  sleep(4.0);
  
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Plan 2
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Plan 2: Set end effector target position for object.");

  geometry_msgs::Pose pose_target2;
  pose_target2.orientation.x = 0.5;
  pose_target2.orientation.y = 0.5;
  pose_target2.orientation.z = -0.5;
  pose_target2.orientation.w = 0.5;
  pose_target2.position.x = box_pose.position.x;
  pose_target2.position.y = box_pose.position.y;
  pose_target2.position.z = box_pose.position.z + 0.3;
  //float a = box_pose.position.z;
  //if (a < 1.0)
  //{pose_target2.position.z = box_pose.position.z + 0.3;}
  //else
  //{pose_target2.position.z = box_pose.position.z + 0.35;}



  
  group.setPoseTarget(pose_target2);
  ROS_INFO("Move end effector to target position");
  group.move();

  sleep(3.0);
  // Attach Object to robot

  ROS_INFO("Attach the object to the robot");

  group.attachObject(collision_object.id);

  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(5.0);

  /* set end effector position */
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Plan 3
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  /* set end effector position */
  ROS_INFO("Plan 3: Set end effector target position in between initial object position and mobile robot position");
  geometry_msgs::Pose pose_target3;
  pose_target3.orientation = pose_target2.orientation;
  pose_target3.position.x = 0.742; //0.742
  pose_target3.position.y = 1.05; //1.05
  pose_target3.position.z = 1.16; //1.16

  group.setPoseTarget(pose_target3);
  ROS_INFO("Plan 3: Move end effector to target position");
  group.move();
  sleep(5.0);


  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Plan 4
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Plan 4: Set end effector target position to mobile robot position + height");
  geometry_msgs::Pose pose_target4;
  pose_target4.orientation = pose_target2.orientation;
  pose_target4.position.x = 0.43;
  pose_target4.position.y = 1.32;
  pose_target4.position.z = 1; //0.80 fuer tiefer

  group.setPoseTarget(pose_target4);
  ROS_INFO("Plan 4: Move end effector to target position");
  group.move();
  sleep(2.0);


  
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //  Detach Object
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  ROS_INFO("Detach the object from robot arm");
  group.detachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object detached. */
  sleep(4.0);

  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Plan 5
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO("Plan 5: Set end effector target position to end state");

  geometry_msgs::Pose pose_target5;

  pose_target5.position.x = 0.8;
  pose_target5.position.y = 0.8;
  pose_target5.position.z = 1.25;

  group.setPoseTarget(pose_target5);
  ROS_INFO("Plan 5: Move end effector to target position");
  group.move();
  sleep(2.0);




  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();
  group.clearPoseTargets();


  ros::shutdown();  
  return 0;
}

