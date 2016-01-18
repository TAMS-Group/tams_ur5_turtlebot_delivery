
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

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  moveit::planning_interface::MoveGroup group("UR5_arm");

  
//transform koordinaden abgreifen
  double posx = 0.5;
  double posy = 0.3;
  

  
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(120);
  group.setNumPlanningAttempts(50);
  
  group.clearPoseTargets();
  group.clearPathConstraints();
  
  
  geometry_msgs::Pose pose_start;
  pose_start.orientation.w = 1;
  pose_start.position.x = 0.8;
  pose_start.position.y = 0.8;
  pose_start.position.z = 1.25;
    
  geometry_msgs::Pose pose_grab;
  pose_grab.orientation.x = 0.5;
  pose_grab.orientation.y = 0.5;
  pose_grab.orientation.z = -0.5;
  pose_grab.orientation.w = 0.5;
  pose_grab.position.x = posx;
  pose_grab.position.y = posy;
  pose_grab.position.z = 1.2;
  
  geometry_msgs::Pose pose_place;
  pose_place.orientation.x = 0.5;
  pose_place.orientation.y = 0.5;
  pose_place.orientation.z = -0.5;
  pose_place.orientation.w = 0.5;
  pose_place.position.x = 0.35;
  pose_place.position.y = 1.35;
  pose_place.position.z = 0.8;
  
  
  geometry_msgs::Pose pose_bottle;
  pose_bottle.orientation.w = 1;
  pose_bottle.position.x = posx;
  pose_bottle.position.y = posy;
  pose_bottle.position.z = 0.885;
  
  
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();
  collision_object.id = "flasche";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.08;
  primitive.dimensions[1] = 0.08;
  primitive.dimensions[2] = 0.28;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose_bottle);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  
  
  ROS_INFO("bewege zu startzustand");
  group.setPoseTarget(pose_start);
  bool success = group.move();
  if(!success) {
    ROS_INFO("FAILED SHUTTING DOWN");
    planning_scene_interface.removeCollisionObjects(object_ids);
    ros::shutdown();  
    return 0;
  }
  ROS_INFO("OK");    
  sleep(2.0);

  ROS_INFO("fuege Flasche ein");
  planning_scene_interface.addCollisionObjects(collision_objects);
  ROS_INFO("OK");
  
  ROS_INFO("bewege zu greifposition");    
  group.setPoseTarget(pose_grab);
  success = group.move();
  if(!success) {
    ROS_INFO("FAILED SHUTTING DOWN");
    planning_scene_interface.removeCollisionObjects(object_ids);
    ros::shutdown();  
    return 0;
  }
  ROS_INFO("OK");    
  sleep(2.0);

  ROS_INFO("Flasche anhaengen");    
  group.attachObject(collision_object.id);
  ROS_INFO("OK");    

  
  ROS_INFO("bewege zu turtle");    
  group.setPoseTarget(pose_place);
  success = group.move();
  if(!success) {
    ROS_INFO("FAILED SHUTTING DOWN");
    planning_scene_interface.removeCollisionObjects(object_ids);
    ros::shutdown();  
    return 0;
  }
  ROS_INFO("OK");    
  sleep(2.0);
  
  ROS_INFO("Flasche abhaengen");    
  group.detachObject(collision_object.id);
  ROS_INFO("OK"); 
  sleep(2.0);
  
  ROS_INFO("bewege zu endzustand");    
  group.setPoseTarget(pose_start);
  success = group.move();
  if(!success) {
    ROS_INFO("FAILED SHUTTING DOWN");
    planning_scene_interface.removeCollisionObjects(object_ids);
    sleep(2.0);
    ros::shutdown();  
    return 0;
  }
  sleep(2.0);

  ROS_INFO("entferne objekte");
  planning_scene_interface.removeCollisionObjects(object_ids);
  sleep(2.0);

  
  ROS_INFO("ENDE");


  ros::shutdown();  
  return 0;
}
