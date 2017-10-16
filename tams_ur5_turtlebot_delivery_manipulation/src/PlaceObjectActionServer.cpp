
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <sensor_msgs/JointState.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/StatePropagator.h>

#include <robotiq_s_model_control/SModel_robot_output.h>

#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <project15_coordination/PlaceObjectAction.h>

#include <iostream>
#include <stdio.h>
#include <math.h>

#define START_POSE  "pour_default"

class PlaceObjectAction
{
protected:

    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<project15_coordination::PlaceObjectAction> actionserver_;
    std::string action_name_;

    project15_coordination::PlaceObjectFeedback feedback_;
    project15_coordination::PlaceObjectResult result_;

    bool success;

    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient get_planning_scene_client;

public:

    PlaceObjectAction(std::string name) :
        actionserver_(node_handle_ , name, boost::bind(&PlaceObjectAction::executeCB, this, _1), false),
        action_name_(name)
    {
        actionserver_.start();
        planning_scene_diff_client = node_handle_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();

        get_planning_scene_client = node_handle_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
        get_planning_scene_client.waitForExistence();
    }

    ~PlaceObjectAction(void)
    {

    }

    void executeCB(const project15_coordination::PlaceObjectGoalConstPtr &goal)
    {
        ros::Rate r(1);

        ros::Duration dur5(5.0);

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::planning_interface::MoveGroup group_arm("arm");
        moveit::planning_interface::MoveGroup group_gripper("gripper");
	group_arm.setSupportSurfaceName("table");
        
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;
        
        ROS_INFO("oeffne hand");
        group_gripper.setNamedTarget("open");
        success = group_gripper.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            failed();
            return;
        }

        group_arm.clearPoseTargets();
        group_arm.clearPathConstraints();


        moveit_msgs::CollisionObject turtle;
        turtle.header.frame_id = "turtlebot";
        turtle.id = "turtle";

        double turtlehoehe = 0.41;

        geometry_msgs::Pose pose_turtle;
        pose_turtle.orientation.w = 1;
        pose_turtle.position.z = turtlehoehe/2;


        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.35;
        primitive.dimensions[1] = 0.35;
        primitive.dimensions[2] = turtlehoehe;

        turtle.primitives.push_back(primitive);
        turtle.primitive_poses.push_back(pose_turtle);
        turtle.operation = turtle.ADD;

        planning_scene.world.collision_objects.push_back(turtle);
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);


        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.ROBOT_STATE;

        get_planning_scene_client.call(request, response);

        if(!similarJointStates(response.scene.robot_state.joint_state, group_arm.getNamedTargetValues(START_POSE))){
            ROS_INFO("bewege zu startzustand");

            group_arm.setNamedTarget(START_POSE);
        
            success = group_arm.move();
            if(!success) {
                ROS_INFO("FAILED SHUTTING DOWN");
                removeObject(turtle);
                failed();
                return;
            }
            ROS_INFO("OK");
            //during the last movement, the camera could have detected the arm as object
            //make sure the vision module has enough time to detect the correct object
            std::cout << "vor dem 5 sec schlafen" << std::endl;
            dur5.sleep();
        }

        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            listener.waitForTransform("/world", "/object", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/world", "/object", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            removeObject(turtle);
            failed();
            return;
        }

        double handoffset = 0.20;

        double beforeoffset = 0.15;
        double tischhoehe = 0.745;
        double abwurfabstand = 0.06;

        double posx = transform.getOrigin().getX();
        double posy = transform.getOrigin().getY();
        double posz = transform.getOrigin().getZ();
        
        double objekthoehe = posz-tischhoehe;

        ROS_INFO("x %f", posx);
        ROS_INFO("y %f", posy);
        ROS_INFO("y %f", posz);

        geometry_msgs::Pose pose_bottle;
        pose_bottle.orientation.w = 1;
        pose_bottle.position.x = posx;
        pose_bottle.position.y = posy;
        pose_bottle.position.z = tischhoehe + (objekthoehe/2.0);

        geometry_msgs::PoseStamped pose_place;
        pose_place.header.frame_id = "turtlebot_holder";
        pose_place.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, M_PI/2, 0.0);
        pose_place.pose.position.z += objekthoehe + abwurfabstand;


        geometry_msgs::PoseStamped pose_in_between;
        pose_in_between.header.frame_id = "turtlebot_holder";
        pose_in_between.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2, M_PI/2, 0.0);
        pose_in_between.pose.position.z += beforeoffset + objekthoehe;


        moveit_msgs::CollisionObject can;
        can.header.frame_id = group_arm.getPlanningFrame();
        can.id = "can";

        shape_msgs::SolidPrimitive primitive_can;
        primitive_can.type = primitive.CYLINDER;
        primitive_can.dimensions.push_back(objekthoehe); //height
        primitive_can.dimensions.push_back(0.04); //radius

        can.primitives.push_back(primitive_can);
        can.primitive_poses.push_back(pose_bottle);
        can.operation = can.ADD;

        planning_scene.world.collision_objects.push_back(can);
        srv.request.scene = planning_scene;

        ROS_INFO("fuege Flasche ein");
        planning_scene_diff_client.call(srv);
        ROS_INFO("OK");

        ROS_INFO("pick object");
        if(!group_arm.planGraspsAndPick("can")){
            ROS_ERROR("pick failed");
            removeObject(turtle);
            removeAttachedObject("can");
            failed();
            return;
        }

        ROS_INFO("bewege zu Zwischenposition");
        group_arm.setPoseTarget(pose_in_between);
        success = group_arm.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            removeObject(turtle);
            removeAttachedObject("can");
            failed();
            return;
        }
        ROS_INFO("OK");

        ROS_INFO("bewege zu turtle");
        group_arm.setPoseTarget(pose_place);
        success = group_arm.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            removeObject(turtle);
            removeAttachedObject("can");
            failed();
            return;
        }
        ROS_INFO("OK");

        ROS_INFO("oeffne hand");
        group_gripper.setNamedTarget("open");
        success = group_gripper.move();
        if(!success) {
            //if hand doesn't open, the hand will move to the position in between 
            //to not distroy the holder for the can, when the turtlebot moves away
            ROS_ERROR("Loading failed, arm moves in a safe position. The node will shut down, please restart.");
            group_arm.setPoseTarget(pose_in_between);
            group_arm.move();
            ros::shutdown();
            failed();
            return;
        }

        ROS_INFO("Flasche abhaengen");
        removeAttachedObject("can");

        ROS_INFO("OK");


        ROS_INFO("bewege zu Zwischenposition");
        group_arm.setPoseTarget(pose_in_between);
        success = group_arm.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            removeObject(turtle);
            removeObject(can);
            failed();
            return;
        }
        ROS_INFO("OK");
        
        result_.status = 0;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        actionserver_.setSucceeded(result_);

        ROS_INFO("bewege zu endzustand");
        group_arm.setNamedTarget(START_POSE);
        success = group_arm.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            removeObject(turtle);
            removeObject(can);
            failed();
            return;
        }

        ROS_INFO("entferne objekte");
        removeObject(turtle);
        removeObject(can);

        ROS_INFO("ENDE");

        //*****************************ROUTINE ENDE**********************************
    }

    void removeObject(moveit_msgs::CollisionObject collision_object){
        moveit_msgs::PlanningScene planning_scene;
        collision_object.operation = collision_object.REMOVE;
        planning_scene.world.collision_objects.push_back(collision_object);
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }

    void removeAttachedObject(std::string object_name){
        moveit_msgs::PlanningScene planning_scene;
        moveit_msgs::AttachedCollisionObject attached_collision_object;

        attached_collision_object.object.id = object_name;
        attached_collision_object.object.operation = attached_collision_object.object.REMOVE;
        planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }

    bool similarJointStates(sensor_msgs::JointState joint_states, std::map<std::string, double> joint_values){
        for(int i=0; i < joint_states.name.size(); i++){
            if(joint_values.find(joint_states.name[i]) != joint_values.end()){
                double rad_diff = fabs(joint_states.position[i] - joint_values.at(joint_states.name[i]));
                if(rad_diff > 0.01){
                    return false;
                }
            }
        }
        return true;
    }

    void failed()
    {
       ROS_INFO("%s: FAILED", action_name_.c_str());
       result_.status = 1;
       actionserver_.setAborted(result_);
    }
    
};

int main(int argc, char** argv)
{
    std::cout << "action server started" << std::endl;
    ros::init(argc, argv, "PlaceObject");
    PlaceObjectAction ur5(ros::this_node::getName());
    ros::spin();

    return 0;
}
