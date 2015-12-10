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
  group.set_planning_time(120)
  
  
  
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
  pose_target.position.x = 0.7
  pose_target.position.y = 0.4
  pose_target.position.z = 1.1

  group.set_pose_target(pose_target,"ee_link")
  #group.set_position_target([0.7,0.4,0.1], end_effector_link)
  print "planning plan1"
  plan1 = group.plan()
  success = group.execute(plan1)
  print "plan1 finished"
  group.clear_pose_targets()
  group.clear_path_constraints()

  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
