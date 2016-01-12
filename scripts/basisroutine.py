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
  group.set_planning_time(400)
  
  
  ########## hand setup
 # hand_pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)
 # close_command = outputMsg.SModel_robot_output()
 # close_command.rPRA = 255
  
 # open_command = outputMsg.SModel_robot_output()
 # open_command.rPRA = 0
  
  rospy.sleep(2)
  
  
  ###########################################################################
  #### PLAN 1
  ###########################################################################
  group.clear_pose_targets()
  group.clear_path_constraints()
  
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  
  pose_target.orientation.x = 0
  pose_target.orientation.y = 0
  pose_target.orientation.z = 0
  pose_target.orientation.w = 1

  pose_target.position.x = 0.8
  pose_target.position.y = 0.8
  pose_target.position.z = 1.25

  group.set_pose_target(pose_target)

  print "planning plan1"
  plan1 = group.plan()
  success = group.execute(plan1)
  print "plan1 finished"
  group.clear_pose_targets()
  group.clear_path_constraints()
  rospy.sleep(1)
  

  ###########################################################################
  #### PLAN 2
  ###########################################################################
  print "============ Generating plan 2"
  pose_target = geometry_msgs.msg.Pose()
  
  pose_target.orientation.x = 0.5
  pose_target.orientation.y = -0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5

  pose_target.position.x = 0.83
  pose_target.position.y = 0.9
  pose_target.position.z = 0.86

  group.set_pose_target(pose_target)

  print "planning plan2"
  plan2 = group.plan()
  success = group.execute(plan2)
  print "plan2 finished"
  group.clear_pose_targets()
  group.clear_path_constraints()
  rospy.sleep(1)

  ######################################################################
  #### Plan 3: Set joint value in desirable state for gripper to grip
  ######################################################################
  print "============ Generating plan 3"
  
  group_variable_values = group.get_current_joint_values()
  group_variable_values[5] = -1.57
  group.set_joint_value_target(group_variable_values)
  plan3 = group.plan()
  success = group.execute(plan3)
  print "plan3 finished"
  rospy.sleep(1)


  ######################################################################
  #### Plan 4: Approach Object
  ######################################################################
  #print "============ Generating plan 4"
  
  #pose_target.orientation.x = 0.5
  #pose_target.orientation.y = -0.5
  #pose_target.orientation.z = -0.5
  #pose_target.orientation.w = 0.5

  #pose_target.position.x = 0.7
  #pose_target.position.y = 0.7
  #pose_target.position.z = 0.9

  #group.set_pose_target(pose_target)

  #constraints = Constraints()
  #constraints.name = "upright"
  #orientation_constraint = OrientationConstraint()
  #orientation_constraint.header.frame_id = group.get_planning_frame()
  #orientation_constraint.link_name = group.get_end_effector_link()
  #orientation_constraint.orientation.x = 0.5
  #orientation_constraint.orientation.y = -0.5
  #orientation_constraint.orientation.z = -0.5
  #orientation_constraint.orientation.w = 0.5
  
  #pi = 3.14159265
  #orientation_constraint.absolute_x_axis_tolerance = 0.2
  #orientation_constraint.absolute_y_axis_tolerance = 0.2
  #orientation_constraint.absolute_z_axis_tolerance = 0.2 #ignore this axis
  #orientation_constraint.weight = 1
  #constraints.orientation_constraints.append(orientation_constraint) 
  
  #group.set_path_constraints(constraints)

  #print "planning plan4"
  #plan4 = group.plan()
  #success = group.execute(plan4)
  #print "plan4 finished"
  #group.clear_pose_targets()
  #group.clear_path_constraints()
  #rospy.sleep(1)

  ######################################################################
  #### Plan 5
  ######################################################################


  #######################################################################
  moveit_commander.roscpp_shutdown()
  #######################################################################
  ## END
  #######################################################################
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
