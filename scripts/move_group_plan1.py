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
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()
  
  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()
  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("manipulator")
  group.set_planner_id("RRTConnectkConfigDefault")
  
  
  group.clear_pose_targets()
  group.clear_path_constraints()
  
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  #pose_target.orientation.x = 0.7071067811865476
  #pose_target.orientation.y = 0
  #pose_target.orientation.z = -0.7071067811865476
  pose_target.orientation.x = 0.5
  pose_target.orientation.y = 0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5
  pose_target.position.x = 0.7
  pose_target.position.y = 0.3
  pose_target.position.z = 1.0

  #pose_target = [x, y, z, rot_x, rot_y, rot_z]
  #pose_targets.append(pose_target)
  #pose_target = [0.4, 1.2, 0.6, 0, 1.57, 0]
  #pose_targets.append(pose_target)
  #pose_target = [0.77, 0.33, 1.0, 0, 1.57, 0]
  group.set_pose_target(pose_target)
  print "planning plan1"
  plan1 = group.plan()
  group.execute(plan1)
  print "plan1 finished"
  
  rospy.sleep(1)
  
  
  group.clear_pose_targets()
  group.clear_path_constraints()
  #
  #
  pose_target = geometry_msgs.msg.Pose()
  #pose_target.orientation.x = 0.7071067811865476
  #pose_target.orientation.y = 0
  #pose_target.orientation.z = -0.7071067811865476
  pose_target.orientation.x = 0.5
  pose_target.orientation.y = 0.5
  pose_target.orientation.z = -0.5
  pose_target.orientation.w = 0.5
  pose_target.position.x = 0.8
  pose_target.position.y = 1.0
  pose_target.position.z = 0.6
 # 
 # group.set_position_target([0.4,1.2,0.6], "ee_link")
  group.set_pose_target(pose_target)
  
  constraints = Constraints()
  constraints.name = "upright"
  orientation_constraint = OrientationConstraint()
  orientation_constraint.header.frame_id = "/world"
  orientation_constraint.link_name = "ee_link"#group.get_end_effector_link()
  
  orientation_constraint.orientation.x = 0.5
  orientation_constraint.orientation.y = 0.5
  orientation_constraint.orientation.z = -0.5
  orientation_constraint.orientation.w = 0.5

  #orientation_constraint.orientation = pose_target.orientation
  #orientation_constraint.orientation.x = -orientation_constraint.orientation.x
  #orientation_constraint.orientation.z = -orientation_constraint.orientation.z
  orientation_constraint.absolute_x_axis_tolerance = 0.1
  orientation_constraint.absolute_y_axis_tolerance = 0.1
  orientation_constraint.absolute_z_axis_tolerance = 3.1 #ignore this axis
  orientation_constraint.weight = 1
  constraints.orientation_constraints.append(orientation_constraint) 
  
  group.set_path_constraints(constraints)
  #print "planning plan1"
  plan1 = group.plan()
  group.execute(plan1)
  #print "plan1 finished"
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