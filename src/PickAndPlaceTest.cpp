#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

class PickAndPlaceTest{

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;

public:
    PickAndPlaceTest(){
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
    }

    ~PickAndPlaceTest(){
    }

    void executePick(){
        moveit::planning_interface::MoveGroup arm("arm");
        spawnObject();
    }

    void spawnObject(){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "/table_top";
        object.id = "object";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.05;
        primitive.dimensions[1] = 0.05;
        primitive.dimensions[2] = 0.20;

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = primitive.dimensions[2]/2;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;
        planning_scene.world.collision_objects.push_back(object);
        
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPTest");
    PickAndPlaceTest testClass;
    testClass.executePick();
    ros::spin();
    return 0;
}
