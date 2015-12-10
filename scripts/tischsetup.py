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

def move_group_python_interface_tutorial():

  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  robot = moveit_commander.RobotCommander()
  
  group = moveit_commander.MoveGroupCommander("UR5_arm")

  scene = moveit_commander.PlanningSceneInterface()

  rospy.sleep(2)
  
  

  pose = geometry_msgs.msg.PoseStamped()
  pose.header.frame_id = group.get_planning_frame()
  pose.pose.orientation.w = 1
  pose.pose.position.x = 0.7
  pose.pose.position.y = 0.4
  pose.pose.position.z = 0.85
  
  
  print "flasche wird hinzugefuegt"
  scene.add_box("flasche3", pose, (0.05,0.05,0.2))
  print "flasche hinzugefuegt OK"
  
  

  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
