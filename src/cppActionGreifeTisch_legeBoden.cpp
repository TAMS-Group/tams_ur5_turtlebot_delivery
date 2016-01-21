
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/StatePropagator.h>

#include <robotiq_s_model_control/SModel_robot_output.h>

#include <actionlib/server/simple_action_server.h>
#include <project15_coordination/placeObject.h>

class UR5Action
{
protected:

  ros::NodeHandle node_handle_;
  actionlib::SimpleActionServer<project15_coordination::placeOject> actionserver_;
  std::string action_name_;
  
  project15_coordination::placeObjectFeedback feedback_;
  project15_coordination::placeObjectResult result_;

public:
  
  UR5Action(std::string name) :
    actionserver_(node_handle_ , name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
    {
      actionserver_.start();
    }
    
    ~UR5Action(void) 
    {
      
    }
    
   void executeCB(const project15_coordination::placeObjectGoalConstPtr &goal)
   {
  
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
     
     ros::Rate r(1);
     bool success = true;
     
     feedback_.sequence.clear();
     feedback_.sequence.push_back(0);
     feedback_.sequence.push_back(1);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  moveit::planning_interface::MoveGroup group("UR5_arm");

  
//transform koordinaden abgreifen
  double posx = 0.5;
  double posy = 0.3;
  
  
 //######### hand setup
  ros::NodeHandle n;
  ros::Publisher hand_pub = n.advertise<robotiq_s_model_control::SModel_robot_output>("SModelRobotOutput", 1000);
  robotiq_s_model_control::SModel_robot_output close_command;
  close_command.rPRA = 255;
  
  robotiq_s_model_control::SModel_robot_output open_command;
  open_command.rPRA = 0;
  
  
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(240);
  group.setNumPlanningAttempts(50);
  
  group.clearPoseTargets();
  group.clearPathConstraints();
  
  
  geometry_msgs::Pose pose_start;
  pose_start.orientation.x = 0.5;
  pose_start.orientation.y = 0.5;
  pose_start.orientation.z = -0.5;
  pose_start.orientation.w = 0.5;
  pose_start.position.x = 0.74;
  pose_start.position.y = 0.56;
  pose_start.position.z = 1.2;
    
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
  r.sleep(2.0);
  
  ROS_INFO("oeffne hand");    
  hand_pub.publish(open_command);
  r.sleep(5.0);

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
  r.sleep(2.0);

  ROS_INFO("Flasche anhaengen");    
  group.attachObject(collision_object.id);
  ROS_INFO("OK");  
  r.sleep(2.0);
  
  ROS_INFO("Schließe hand");    
  hand_pub.publish(close_command);  
  r.sleep(5.0);
  
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
  r.sleep(2.0);
  
  ROS_INFO("Flasche abhaengen");    
  group.detachObject(collision_object.id);
  ROS_INFO("OK"); 
  r.sleep(2.0);
  
  ROS_INFO("oeffne hand");    
  hand_pub.publish(open_command);
  r.sleep(5.0);
  
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
  r.sleep(2.0);

  ROS_INFO("entferne objekte");
  planning_scene_interface.removeCollisionObjects(object_ids);
  r.sleep(2.0);

  
  ROS_INFO("ENDE");


  ros::shutdown();  
  return 0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armroutine");
  FibonacciAction fibonacci(ros::this_node::getName());
  ros::spin();

  return 0;
}
