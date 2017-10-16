
import rospy
import actionlib
from actionlib import GoalStatus

from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


class Turtle(object):
    def __init__(self, name, x, y):
        self.name = name
        self.namespace = name + "/"
        self.home_x = x
        self.home_y = y
        self.current_move_base_goal = None
        self.in_front_of_home = False
        self.docked = True
        self.has_task = False
        self.undock_time = None
        self.waiting_time = None
        self.soundhandle = SoundClient()

        self.moveBaseClient = actionlib.SimpleActionClient(self.namespace + 'move_base', MoveBaseAction)
        self.dockingClient = actionlib.SimpleActionClient(self.namespace + 'dock_drive_action', AutoDockingAction)
        self.velocity_publisher = rospy.Publisher(self.namespace + 'cmd_vel_mux/input/teleop', Twist, queue_size=1)
        if not self.moveBaseClient.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("still waiting for "+self.namespace+"move_base")
            self.moveBaseClient.wait_for_server()
        if not self.dockingClient.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("still waiting for "+self.namespace+"dock_drive_action")
            self.dockingClient.wait_for_server()
        # TODO: check battery state

    def go_to_loading(self):
        """
        move turtle near package loading station
        :return: True if arrived
        """
        return self.goto_goal(0, 0)

    def go_to_waiting(self):
        """
        move turtle to waiting position
        :return:
        """
        if self.in_front_of_home:
            if self.dock():
                self.in_front_of_home = False
        elif self.goto_goal(self.home_x, self.home_y):
            self.in_front_of_home = True


    def dock(self):
        """
        dock to nearby power loading station
        :return: True if succeeded
        """
        if not self.dockingClient.gh or not self.dockingClient.get_state() in (GoalStatus.SUCCEEDED, GoalStatus.PENDING, GoalStatus.ACTIVE):
            self.dockingClient.send_goal(AutoDockingGoal()) #TODO test if parameter is required
            rospy.loginfo(self.name + ": docking") 
        if self.dockingClient.get_state() == GoalStatus.SUCCEEDED:  
            self.dockingClient.stop_tracking_goal()
            rospy.loginfo(self.name + ": docking succeeded")
            self.docked = True    
            return True
        return False

    def goto_goal(self, x, y):
        """
        Drive to a position

        :param x:
        :param y:
        :return: true if reached
        """
        pose = (x,y)
        if self.dockingClient.gh:
            return False
        if self.docked == True:
            self.undock()
        if self.current_move_base_goal != pose:
            self.moveBaseClient.cancel_goal()
            self.current_move_base_goal = None
        if self.current_move_base_goal == pose:
            if self.moveBaseClient.get_state() == GoalStatus.SUCCEEDED:
                self.moveBaseClient.stop_tracking_goal()
                rospy.loginfo(self.name + ": finished driving")
                return True
            if not self.moveBaseClient.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                self.call_move_base(x,y)
            return False
        self.current_move_base_goal = pose
        self.call_move_base(x,y)

    def undock(self):
        if self.docked == True:
            if not self.undock_time:
                self.undock_time = rospy.Time.now()
            vel = Twist()
            if self.undock_time + rospy.Duration(3.0) >= rospy.Time.now():
                vel.linear.x = -0.2
                self.velocity_publisher.publish(vel)
                return False    
            else:
                vel.linear.x = 0.0
                self.velocity_publisher.publish(vel)
                self.docked = False
                self.undock_time = None
                return True

    def call_move_base(self, x, y):
        """
        specify new goal for move_base of turtle
        :param x:
        :param y:
        :return:
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.z = 1
        self.moveBaseClient.send_goal(goal)
        rospy.loginfo(self.name + ": driving")

    def wait_at_goal(self):
        if not self.waiting_time:
            self.waiting_time = rospy.Time.now()
            self.soundhandle.say("please pick up your chips", 'voice_kal_diphone')
        if self.waiting_time + rospy.Duration(15.0) < rospy.Time.now():
            return True
        return False

    def __repr__(self):
        return "<Turtle %s>" % self.name
