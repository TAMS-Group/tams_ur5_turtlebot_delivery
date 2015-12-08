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
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg

def move_group_python_interface_tutorial():

  
  
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)


  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  group = moveit_commander.MoveGroupCommander("manipulator")
  group.set_planner_id("RRTConnectkConfigDefault")
  group.set_planning_time(120)
  
 
  
  rospy.sleep(2)
  
  
  group.clear_pose_targets()
  group.clear_path_constraints()
  
 
  group.pick("flasche3")


  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
