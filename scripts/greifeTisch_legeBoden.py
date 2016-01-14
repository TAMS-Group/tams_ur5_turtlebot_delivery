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

  print "============ Starting"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('armroutine',
                  anonymous=True)

  #### transform koordinaden abgreifen
  posx = 0.5
  posy = 0.3
  

  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  group = moveit_commander.MoveGroupCommander("UR5_arm")
  group.set_planner_id("RRTConnectkConfigDefault")
  group.set_planning_time(30)
  
  
  ########## hand setup
 # hand_pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)
 # close_command = outputMsg.SModel_robot_output()
 # close_command.rPRA = 255
  
 # open_command = outputMsg.SModel_robot_output()
 # open_command.rPRA = 0

##################################################
#### Plan 1: Go to start position
##################################################
  pose_target = geometry_msgs.msg.Pose()
  print "============ Plan 1: Go to start position"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.position.x = 0.8
  pose_target.position.y = 0.8
  pose_target.position.z = 1.25
  
  
  
  group.set_pose_target(pose_target)
  print "planning plan1"
  group.go()
  print "plan1 finished"
  group.clear_pose_targets()
  group.clear_path_constraints()
  rospy.sleep(1)
  
  
  
##################################################
#### fuege flasche ein
##################################################
  pose = geometry_msgs.msg.PoseStamped()
  pose.header.frame_id = group.get_planning_frame()
  pose.pose.orientation.w = 1
  pose.pose.position.x = posx
  pose.pose.position.y = posy
  pose.pose.position.z = 0.885
  print "flasche wird hinzugefuegt"
  scene.add_box("flasche3", pose, (0.08,0.08,0.28))
  print "flasche hinzugefuegt OK"
  
  
################################################  
#### Plan 2: Move to object position to grab
################################################
  print "Plan 2: Move to object in order to grab"
  group.set_start_state_to_current_state()

  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = 0.5
  pose_target.orientation.y = 0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5 

  pose_target.position.x = posx
  pose_target.position.y = posy
  pose_target.position.z = 1.2
 # 
 # group.set_position_target([0.4,1.2,0.6], "ee_link")
  group.set_pose_target(pose_target)
  


#################
# hand_pub.publish(close_command)
  
  print "planning plan2"
  #group.set_start_state(robot.get_current_state())
  rospy.sleep(2)
  group.go()
  print "plan2 finished2"

  group.attach_object("flasche3")
  print "object attached"
  
  group.clear_pose_targets()
  group.clear_path_constraints()


################################################  
#### Plan 3: Move to in-between-position (table,turtle)
################################################




################################################  
#### Plan 4: Go to turtle
################################################
  print "Plan 4: Move to turtle"

  #group.set_start_state(robot.get_current_state())
  #constraints = Constraints()
  #constraints.name = "upright"
  #orientation_constraint = OrientationConstraint()
  #orientation_constraint.header.frame_id = group.get_planning_frame()
  #orientation_constraint.link_name = group.get_end_effector_link()
  #orientation_constraint.orientation.x = 0.5
  #orientation_constraint.orientation.y = 0.5
  #orientation_constraint.orientation.z = -0.5
  #orientation_constraint.orientation.w = 0.5
  
  #pi = 3.14159265
  #orientation_constraint.absolute_x_axis_tolerance = 0.9
  #orientation_constraint.absolute_y_axis_tolerance = 0.9
  #orientation_constraint.absolute_z_axis_tolerance = 0.9 #pi =ignore this axis
  #orientation_constraint.weight = 1
  #constraints.orientation_constraints.append(orientation_constraint) 
  
  #group.set_path_constraints(constraints)

  pose_target.orientation.x = 0.5
  pose_target.orientation.y = 0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5 

  pose_target.position.x = 0.35
  pose_target.position.y = 1.35
  pose_target.position.z = 0.8 ## 0.8 fuer tiefer

  group.set_pose_target(pose_target)

  print "planning plan4"
  #group.set_start_state(robot.get_current_state())
  rospy.sleep(2)
  group.go()
  print "plan4 finished"

  group.clear_pose_targets()
  group.clear_path_constraints()


  #################################################
  rospy.sleep(2)

  group.detach_object("flasche3")
  print "object detached"
 # hand_pub(open_command)
  rospy.sleep(2)
  group.set_start_state_to_current_state()
  rospy.sleep(2)
  ##############################################################
  
################################################  
#### Plan 5: Go back to start position
################################################
  print "Plan 5: Go back to start position"

  pose_target = geometry_msgs.msg.Pose()
  print "============ Generating plan 5"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.position.x = 0.8
  pose_target.position.y = 0.8
  pose_target.position.z = 1.25
  
  

  group.set_pose_target(pose_target)

  print "planning plan5"
  group.go()
  print "plan5 finished"
  group.clear_pose_targets()
  group.clear_path_constraints()
  rospy.sleep(1)


  scene.remove_world_object("flasche3")
  print "object removed from scene"

  


  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
