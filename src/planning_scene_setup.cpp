
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <geometric_shapes/shape_operations.h>
#include <boost/filesystem.hpp>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


int main(int argc, char **argv)
{
  ros::init (argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_time(10.0);
  sleep_time.sleep();
  sleep_time.sleep();
  

// BEGIN_TUTORIAL
// 
// ROS API
// ^^^^^^^
// The ROS API to the planning scene publisher is through a topic interface
// using "diffs". A planning scene diff is the difference between the current 
// planning scene (maintained by the move_group node) and the new planning 
// scene desired by the user. 

// Advertise the required topic
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Note that this topic may need to be remapped in the launch file 
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
  
  
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "wrist2";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "wrist2";
  /* The id of the object */
  attached_object.object.id = "box";
  
  
  
  
  
  
  boost::filesystem::path path(boost::filesystem::current_path());
  
  
  /* The header must contain a valid TF frame*/
  
  
  shapes::Mesh* m = shapes::createMeshFromResource("file://"+path.string()+"../../project15_gazebo/Media/models/Rest.dae");
  //shapes::createMeshFromResource("file://"+path.string()+"Rest.dae");

  ROS_INFO("mesh loaded");

  
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;  
  shapes::constructMsgFromShape(m, co_mesh_msg);    
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);  
  
  moveit_msgs::CollisionObject co;
  co.meshes.resize(1);
  co.mesh_poses.resize(1);
  co.meshes[0] = co_mesh;
  co.header.frame_id = "kA";
  co.id = "world";
  
  
  co.meshes.push_back(co_mesh);
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;

  
  
  


// Add an object into the environment
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Add the object into the environment by adding it to 
// the set of collision objects in the "world" part of the 
// planning scene. Note that we are using only the "object" 
// field of the attached_object message here.
  ROS_INFO("Adding the object into the world at the location of the right wrist.");
  moveit_msgs::PlanningScene planning_scene;
  
  
  planning_scene.world.collision_objects.push_back(co);
  
  
  
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_time.sleep();
  
  


// Attach an object to the robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// When the robot picks up an object from the environment, we need to 
// "attach" the object to the robot so that any component dealing with 
// the robot model knows to account for the attached object, e.g. for
// collision checking.

// Attaching an object requires two operations
//  * Removing the original object from the environment
//  * Attaching the object to the robot

  /* First, define the REMOVE object message*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "odom_combined";
  remove_object.operation = remove_object.REMOVE;

// Note how we make sure that the diff message contains no other
// attached objects or collisions objects by clearing those fields
// first.
  /* Carry out the REMOVE + ATTACH operation */
  ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_time.sleep();

// Detach an object from the robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Detaching an object from the robot requires two operations
//  * Detaching the object from the robot
//  * Re-introducing the object into the environment

  /* First, define the DETACH object message*/
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "wrist2";
  detach_object.object.operation = attached_object.object.REMOVE;

// Note how we make sure that the diff message contains no other
// attached objects or collisions objects by clearing those fields
// first.
  /* Carry out the DETACH + ADD operation */
  ROS_INFO("Detaching the object from the robot and returning it to the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_time.sleep();

// REMOVE THE OBJECT FROM THE COLLISION WORLD
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Removing the object from the collision world just requires
// using the remove object message defined earlier. 
// Note, also how we make sure that the diff message contains no other
// attached objects or collisions objects by clearing those fields
// first.
  ROS_INFO("Removing the object from the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);
// END_TUTORIAL

  ros::shutdown();
  return 0;
}
