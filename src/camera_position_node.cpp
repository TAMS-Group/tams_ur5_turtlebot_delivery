#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <apriltags_ros/AprilTagDetectionArray.h>


void pose_detection(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{

  tf::TransformBroadcaster br;
  tf::StampedTransform transform;
  tf::StampedTransform tmpTransform;
  
  transform.setOrigin(tf::Vector3(0,0,0));

  tf::TransformListener listener;

  ros::Rate rate(10.0); 
  try {
    listener.waitForTransform("/world", "/floor", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/world", "/floor", ros::Time(0), transform);
    
    listener.waitForTransform("/floor", "/wall", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/floor", "/wall",  ros::Time(0), tmpTransform);
    transform *= tmpTransform;    

    listener.waitForTransform("/wall", "/april_tag_ur5", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/wall", "/april_tag_ur5",  ros::Time(0), tmpTransform);
    transform *= tmpTransform;

    listener.waitForTransform("/tag_0", "/camera_link", ros::Time(0), ros::Duration(10.0) );    
    listener.lookupTransform("/tag_0", "/camera_link",  ros::Time(0), tmpTransform);
    transform *= tmpTransform;

  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/camera_link"));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "camera_position_node");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("tag_detections", 1000, pose_detection);
  ros::spin();
  return 0;
};
