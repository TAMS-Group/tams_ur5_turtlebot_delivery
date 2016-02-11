
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/StatePropagator.h>

#include <robotiq_s_model_control/SModel_robot_output.h>

#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <project15_coordination/PlaceObjectAction.h>

#include <iostream>
#include <stdio.h>


class PlaceObjectAction
{
protected:

    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<project15_coordination::PlaceObjectAction> actionserver_;
    std::string action_name_;

    project15_coordination::PlaceObjectFeedback feedback_;
    project15_coordination::PlaceObjectResult result_;
    
    ros::Publisher hand_pub;

public:

    PlaceObjectAction(std::string name) :
        actionserver_(node_handle_ , name, boost::bind(&PlaceObjectAction::executeCB, this, _1), false),
        action_name_(name)
    {
        actionserver_.start();
	hand_pub = node_handle_.advertise<robotiq_s_model_control::SModel_robot_output>("SModelRobotOutput", 10);
    }

    ~PlaceObjectAction(void)
    {

    }

    void executeCB(const project15_coordination::PlaceObjectGoalConstPtr &goal)
    {
        ros::Rate r(1);

	ros::Duration dur2(2.0);
	ros::Duration dur5(5.0);
        //********************************ROUTINE**********************************

//	ROS_INFO("greife: %s", goal->obj);


        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::planning_interface::MoveGroup group("UR5_arm");

//######### hand setup
	robotiq_s_model_control::SModel_robot_output activate_command;
        activate_command.rACT = 1;
        activate_command.rGTO = 1;
        activate_command.rPRA = 0;
        activate_command.rSPA = 255;
        activate_command.rFRA = 130;
        robotiq_s_model_control::SModel_robot_output close_command;
        close_command.rACT = 1;
        close_command.rGTO = 1;
	close_command.rICS = 1;
        close_command.rPRA = 200;
        close_command.rSPA = 100;
        close_command.rFRA = 130;
	close_command.rPRS = 255;
	close_command.rSPS = 255;
        robotiq_s_model_control::SModel_robot_output open_command;
        open_command.rACT = 1;
        open_command.rGTO = 1;
	open_command.rICS = 1;
        open_command.rPRA = 0;
        open_command.rSPA = 255;
        open_command.rFRA = 130;
	open_command.rPRS = 0;
	open_command.rSPS = 255;
	
	dur2.sleep();
	ROS_INFO("aktiviere hand");
	hand_pub.publish(open_command);
	std::cout << "vor dem 12 sec schlafen" << std::endl;
	dur5.sleep();
	dur5.sleep();
	dur2.sleep();
	std::cout << "nach dem 12 sec schlafen" << std::endl;


        ROS_INFO("oeffne hand");
        hand_pub.publish(open_command);
	std::cout << "vor dem 5 sec schlafen" << std::endl;
	dur5.sleep();
	std::cout << "nach dem 5 sec schlafen" << std::endl;

        group.setPlannerId("RRTConnectkConfigDefault");
        group.setPlanningTime(60);
        group.setNumPlanningAttempts(10);

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



	double turtlehoehe = 0.42;

        geometry_msgs::Pose pose_turtle;
        pose_turtle.orientation.w = 1;
        pose_turtle.position.x = 0.35;
        pose_turtle.position.y = 1.35;
        pose_turtle.position.z = 0.21;



        moveit_msgs::CollisionObject collision_turtle;
        collision_turtle.header.frame_id = group.getPlanningFrame();
        collision_turtle.id = "turtle";


        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.35;
        primitive.dimensions[1] = 0.35;
        primitive.dimensions[2] = turtlehoehe;

        collision_turtle.primitives.push_back(primitive);
        collision_turtle.primitive_poses.push_back(pose_turtle);
        collision_turtle.operation = collision_turtle.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_turtle);

        std::vector<std::string> object_ids;
        object_ids.push_back(collision_turtle.id);



        ROS_INFO("bewege zu startzustand");
        group.setPoseTarget(pose_start);
        bool success = group.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            planning_scene_interface.removeCollisionObjects(object_ids);
	    dur2.sleep();
            failed();
            return;
        }
        ROS_INFO("OK");
	dur5.sleep();
	dur5.sleep();




        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            listener.waitForTransform("/world", "/object", ros::Time(0), ros::Duration(20.0) );
            listener.lookupTransform("/world", "/object", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
	    dur2.sleep();
            failed();
            return;
        }

        //double handoffset = 0.19;
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

//  double posx = 0.93;
//  double posy = 0.32;


        geometry_msgs::Pose pose_before_grip;
        pose_before_grip.orientation.x = 0.5;
        pose_before_grip.orientation.y = 0.5;
        pose_before_grip.orientation.z = -0.5;
        pose_before_grip.orientation.w = 0.5;
        pose_before_grip.position.x = posx;
        pose_before_grip.position.y = posy;
        //pose_before_grip.position.z = 1.3;
        pose_before_grip.position.z = beforeoffset+posz+handoffset;




        geometry_msgs::Pose pose_grab;
        pose_grab.orientation.x = 0.5;
        pose_grab.orientation.y = 0.5;
        pose_grab.orientation.z = -0.5;
        pose_grab.orientation.w = 0.5;
        pose_grab.position.x = posx;
        pose_grab.position.y = posy;
       // pose_grab.position.z = 1.19;
       pose_grab.position.z = posz+handoffset;


        geometry_msgs::Pose pose_bottle;
        pose_bottle.orientation.w = 1;
        pose_bottle.position.x = posx;
        pose_bottle.position.y = posy;
        //pose_bottle.position.z = 0.885;
        pose_bottle.position.z = tischhoehe + (objekthoehe/2.0);


	
	
	geometry_msgs::Pose pose_place;
        pose_place.orientation.x = 0.5;
        pose_place.orientation.y = 0.5;
        pose_place.orientation.z = -0.5;
        pose_place.orientation.w = 0.5;
        pose_place.position.x = 0.315;
        pose_place.position.y = 1.310;
        //pose_place.position.z = 0.935;
        pose_place.position.z = turtlehoehe + objekthoehe + handoffset + abwurfabstand;


        geometry_msgs::Pose pose_in_between;
        pose_in_between.orientation.x = 0.5;
        pose_in_between.orientation.y = 0.5;
        pose_in_between.orientation.z = -0.5;
        pose_in_between.orientation.w = 0.5;
        pose_in_between.position.x = 0.315;
        pose_in_between.position.y = 1.310;
	//pose_in_between.position.z = 1.15;
        pose_in_between.position.z = beforeoffset+turtlehoehe+objekthoehe+handoffset;


        moveit_msgs::CollisionObject collision_flasche;
        collision_flasche.header.frame_id = group.getPlanningFrame();
        collision_flasche.id = "flasche";

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.08;
        primitive.dimensions[1] = 0.08;
        //primitive.dimensions[2] = 0.28;
	primitive.dimensions[2] = objekthoehe;

        collision_flasche.primitives.push_back(primitive);
        collision_flasche.primitive_poses.push_back(pose_bottle);
        collision_flasche.operation = collision_flasche.ADD;

        collision_objects.push_back(collision_flasche);
        object_ids.push_back(collision_flasche.id);




        ROS_INFO("fuege Flasche ein");
        planning_scene_interface.addCollisionObjects(collision_objects);
        ROS_INFO("OK");
        dur2.sleep();



        ROS_INFO("greifposition vorbereiten");
        group.setPoseTarget(pose_before_grip);
        success = group.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            planning_scene_interface.removeCollisionObjects(object_ids);
	    dur2.sleep();
            failed();
            return;
        }
        ROS_INFO("OK");



        ROS_INFO("bewege zu greifposition");
        group.setPoseTarget(pose_grab);
        success = group.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            planning_scene_interface.removeCollisionObjects(object_ids);
	    dur2.sleep();
            failed();
            return;
        }
        ROS_INFO("OK");


        ROS_INFO("Flasche anhaengen");
        std::vector<std::string> touch_links;
        touch_links.push_back("finger_1_link_0");
        touch_links.push_back("finger_1_link_1");
        touch_links.push_back("finger_1_link_2");
        touch_links.push_back("finger_1_link_3");
        touch_links.push_back("finger_2_link_0");
        touch_links.push_back("finger_2_link_1");
        touch_links.push_back("finger_2_link_2");
        touch_links.push_back("finger_2_link_3");
        touch_links.push_back("finger_middel_link_0");
        touch_links.push_back("finger_middle_link_1");
        touch_links.push_back("finger_middle_link_2");
        touch_links.push_back("finger_middle_link_3");
        //std::vector<std::string> touch_links = {"finger_1_link_0", "finger_1_link_1", "finger_1_link_2", "finger_1_link_3", "finger_2_link_0", "finger_2_link_1", "finger_2_link_2", "finger_2_link_3","finger_middel_link_0", "finger_middel_link_1", "finger_middel_link_2", "finger_middel_link_3"};
        group.attachObject(collision_flasche.id, "", touch_links);
        ROS_INFO("OK");
        dur2.sleep();


        ROS_INFO("Schließe hand");
        hand_pub.publish(close_command);
        dur5.sleep();
        dur5.sleep();


        ROS_INFO("bewege zu Zwischenposition");
        group.setPoseTarget(pose_in_between);
        success = group.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            group.detachObject(collision_flasche.id);
            planning_scene_interface.removeCollisionObjects(object_ids);
	    dur2.sleep();
            failed();
            return;
        }
        ROS_INFO("OK");

        ROS_INFO("bewege zu turtle");
        group.setPoseTarget(pose_place);
        success = group.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            group.detachObject(collision_flasche.id);
            planning_scene_interface.removeCollisionObjects(object_ids);
	    dur2.sleep();
            failed();
            return;
        }
        ROS_INFO("OK");

        ROS_INFO("Flasche abhaengen");
        group.detachObject(collision_flasche.id);
        ROS_INFO("OK");
        dur2.sleep();

        ROS_INFO("oeffne hand");
        hand_pub.publish(open_command);
        dur5.sleep();

        ROS_INFO("bewege zu Zwischenposition");
        group.setPoseTarget(pose_in_between);
        success = group.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            group.detachObject(collision_flasche.id);
            planning_scene_interface.removeCollisionObjects(object_ids);
            failed();
            return;
        }
        ROS_INFO("OK");
	
	result_.status = 0;
	ROS_INFO("%s: Succeeded", action_name_.c_str());
	// set the action state to succeeded
	actionserver_.setSucceeded(result_);

        ROS_INFO("bewege zu endzustand");
        group.setPoseTarget(pose_start);
        success = group.move();
        if(!success) {
            ROS_INFO("FAILED SHUTTING DOWN");
            planning_scene_interface.removeCollisionObjects(object_ids);
	    dur2.sleep();
	    //failed();
            return;
        }

        ROS_INFO("entferne objekte");
        planning_scene_interface.removeCollisionObjects(object_ids);
        dur2.sleep();


        ROS_INFO("ENDE");

        //*****************************ROUTINE ENDE**********************************
        

        
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
