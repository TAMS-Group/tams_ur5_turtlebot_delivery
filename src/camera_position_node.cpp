#include <ros/ros.h>
//#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
//#include <apriltags_ros/apriltag_detector.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
//#include <geometry_msgs/Twist.h>
//#include <turtlesim/Spawn.h>



//void pose_detection(const std_msgs::String::ConstPtr& msg)
void pose_detection(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->detections[0].pose.pose.position.x);


   tf::TransformBroadcaster br;
  tf::Transform transform;

	transform.inverse();
/*  transform.setOrigin( tf::Vector3(msg->detections[0].pose.pose.position.x, 
	msg->detections[0].pose.pose.position.y, 
	msg->detections[0].pose.pose.position.z) );

  tf::Quaternion q;
  //q.setRPY(msg->detections[0].pose.pose.orientation.x, 
//	msg->detections[0].pose.pose.orientation.y, 
//	msg->detections[0].pose.pose.orientation.z);
 q.setRPY(0,0,0);

 transform.setRotation(q);
*/
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/tag_0", "/camera_rgb_optical_frame"));
}


int main(int argc, char** argv){
  std::cout << "Bin da";

  ros::init(argc, argv, "camera_position_node");

  ros::NodeHandle node;




  ros::Subscriber sub = node.subscribe("tag_detections", 1000, pose_detection);

/*

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
*/
  //ros::Publisher turtle_vel =
  //  node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
/*
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/turtle2", "/turtle1",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
*/
    //geometry_msgs::Twist vel_msg;
    //vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
    //                                transform.getOrigin().x());
    //vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
    //                              pow(transform.getOrigin().y(), 2));
    //turtle_vel.publish(vel_msg);

    ros::spin();
  //}
  return 0;
};
