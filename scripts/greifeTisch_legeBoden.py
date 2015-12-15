#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL
from std_msgs.msg import String
from moveit_msgs.msg._Constraints import Constraints
from moveit_msgs.msg._OrientationConstraint import OrientationConstraint
from pyassimp.structs import Scene

#import roslib; roslib.load_manifest('robotiq_s_model_control')
#from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg

def move_group_python_interface_tutorial():

  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)


  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  group = moveit_commander.MoveGroupCommander("UR5_arm")
  group.set_planner_id("RRTConnectkConfigDefault")
  group.set_planning_time(240)
  
  
  ########## hand setup
 # hand_pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)
 # close_command = outputMsg.SModel_robot_output()
 # close_command.rPRA = 255
  
 # open_command = outputMsg.SModel_robot_output()
 # open_command.rPRA = 0
  
  rospy.sleep(2)
  
  
  
  group.clear_pose_targets()
  group.clear_path_constraints()
  
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  
  pose_target.orientation.x = 0.5
  pose_target.orientation.y = 0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5 
#pose_target.orientation.w = 1
  pose_target.position.x = 0.7
  pose_target.position.y = 0.4
  pose_target.position.z = 1.1
  
  

  group.set_pose_target(pose_target)
  #group.set_position_target([0.7,0.4,0.1], end_effector_link)
  print "planning plan1"
  plan1 = group.plan()
  success = group.execute(plan1)
  print "plan1 finished"
  group.clear_pose_targets()
  group.clear_path_constraints()
  rospy.sleep(1)
  
  #######################################################################
  rospy.sleep(2)
 # hand_pub.publish(close_command)
  #group.attach_object("flasche3")
  rospy.sleep(2)
  #######################################################################

  pose_target = geometry_msgs.msg.Pose()

  pose_target.orientation.x = 0.5
  pose_target.orientation.y = 0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5 
  #pose_target.orientation.w = 1
  pose_target.position.x = 0.8
  pose_target.position.y = 1.0
  pose_target.position.z = 0.6
 # 
 # group.set_position_target([0.4,1.2,0.6], "ee_link")
  group.set_pose_target(pose_target)
  
  
  constraints = Constraints()
  constraints.name = "upright"
  orientation_constraint = OrientationConstraint()
  orientation_constraint.header.frame_id = group.get_planning_frame()
  orientation_constraint.link_name = group.get_end_effector_link()
  orientation_constraint.orientation.x = 0.5
  orientation_constraint.orientation.y = 0.5
  orientation_constraint.orientation.z = -0.5
  orientation_constraint.orientation.w = 0.5
  
  pi = 3.14159265
  orientation_constraint.absolute_x_axis_tolerance = pi
  orientation_constraint.absolute_y_axis_tolerance = pi
  orientation_constraint.absolute_z_axis_tolerance = pi #ignore this axis
  orientation_constraint.weight = 1
  constraints.orientation_constraints.append(orientation_constraint) 
  
#  group.set_path_constraints(constraints)
  print "planning plan1"
  plan1 = group.plan()
  succes = group.execute(plan1)
  print "plan1 finished"
  
  group.clear_pose_targets()
  group.clear_path_constraints()
  
  #################################################
  rospy.sleep(2)
  #group.detach_object("flasche3")
 # hand_pub(open_command)
  rospy.sleep(2)
  ##############################################################
  
  #rospy.sleep(2)
  #scene.remove_world_object("flasche3")


  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
