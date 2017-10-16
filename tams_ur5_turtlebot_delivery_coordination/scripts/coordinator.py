#!/usr/bin/env python
import rospy
import actionlib

from task import Task, TASK_STATE_DELIVER, TASK_STATE_FETCH_TURTLE, TASK_STATE_GET_ITEM, TASK_STATE_INITIAL, \
    TASK_STATE_POSITION_TURTLE, \
    TASK_STATE_GOAL
from turtle import Turtle
from collections import deque

from tams_ur5_turtlebot_delivery_turtlebot.msg import itemRequest, OnlineTurtle
from tams_ur5_turtlebot_delivery_coordination.msg import PlaceObjectAction, PlaceObjectGoal, taskStatus
from tams_ur5_turtlebot_delivery_coordination.msg import task as task_msg

from geometry_msgs.msg import Pose

ARM_WAITING = 0
ARM_LOADING = 1
ARM_PLACED = 2
ARM_PLACE_FAILED = 3

class coordinator(object):
    def __init__(self):
        self.tasks = deque()
        self.tasksAssigned = list()
        self.turtleBots = list()
        self.armState = ARM_WAITING
        rospy.Subscriber("/itemRequest", itemRequest, self.addTask)

    def addTask(self, item_request):
        """
        This method adds a Task to the taskque
        :param item_request: a itemRequest ros message
        :return:
        """
        t = Task(item_request.pose, item_request.item, item_request.uuid)
        rospy.loginfo("New Task: " + str(t))
        self.tasks.append(t)

    # in case new TurtleBot is discovered
    def addTurtleBot(self, bot):
        rospy.loginfo("Added Turtle: " + str(bot))
        self.turtleBots.append(bot)

    # in case TurtleBot didn't send status for too long
    # remove from list so it won't be assigned tasks
    def removeTurtleBot(self, bot):
        self.turtleBots.remove(bot)

    # should be used as callback function from a
    # subscriber node periodically collecting data
    # (subscriber triggered every time data is received?)
    def updateTurtleBotStatus(self, data):
        for turtle in self.turtleBots:
            pass
            # (check for aliveness at this point or periodically?) self.removeTurtleBot()
            # create new bot if not present
            # read Bot specific values out of data and update if necessary

    def assignTask(self):
        if not self.tasks:
            return
        for turtle in self.turtleBots:
            # choose the turtle for the task
            # TODO: informed selection. For now choose the first one
            if not turtle.has_task:
                currentTask = self.tasks.popleft()
                currentTask.assign_turtle(turtle)
                currentTask.state = TASK_STATE_FETCH_TURTLE
                self.tasksAssigned.append(currentTask)
                # only assign one task
                break

    def doneCB(self, state, result):
        if result.status == 0:
            rospy.loginfo("received item")
            self.armState = ARM_PLACED
        else:
            #TODO: error handling
            rospy.loginfo("put item in region of interest")
            rospy.sleep(rospy.Duration(10.0))
            self.armState = ARM_PLACE_FAILED

    def main(self):
        rospy.init_node('coordinator', anonymous=False)
        # subscribe to requests
        task_status_publisher = rospy.Publisher('/taskStatus', taskStatus, queue_size=1)

        #create Proxy to placeActionServer of arm
        client = actionlib.SimpleActionClient('PlaceObject', PlaceObjectAction)
        client.wait_for_server()

        rospy.loginfo("Waiting for a turtle...")
        # TODO: callback to register new turtles instead of waiting for exactly one
        robotmsg = rospy.wait_for_message("/turtles", OnlineTurtle, timeout=None)
        t = Turtle(robotmsg.name, 2.466515241951, 5.809623560960491)
        self.addTurtleBot(t) # -1.23849964142, 4.34559011459))


        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.assignTask()
            for task in self.tasksAssigned[:]:
                if task.state is TASK_STATE_FETCH_TURTLE:
                    if task.turtle.go_to_loading():
                        task.state = TASK_STATE_POSITION_TURTLE
                if task.state is TASK_STATE_POSITION_TURTLE:
                    # TODO: mutually lock docking bay
                    if task.turtle.dock():
                        rospy.loginfo(task.turtle.name + ": docked")
                        task.state = TASK_STATE_GET_ITEM
                        goal = PlaceObjectGoal(obj=task.item)
                        client.send_goal(goal, self.doneCB)
                        self.armState = ARM_LOADING
                        rospy.loginfo(task.turtle.name + ": waiting for item to be loaded")
                if task.state is TASK_STATE_GET_ITEM:
                    if self.armState is ARM_PLACED:
                        task.state = TASK_STATE_DELIVER
                        self.armState = ARM_WAITING
                        rospy.loginfo(task.turtle.name + ": deliviering")
                    elif self.armState is ARM_PLACE_FAILED:
                        goal = PlaceObjectGoal(obj=task.item)
                        client.send_goal(goal, self.doneCB)
                        self.armState = ARM_LOADING
                if task.state is TASK_STATE_DELIVER:
                    if task.turtle.goto_goal(task.getGoal().position.x, task.getGoal().position.y):
                        task.state = TASK_STATE_GOAL
                        rospy.loginfo(task.turtle.name + ": delivered")
                        rospy.loginfo(task.turtle.name + ": wait at goal")
                if task.state is TASK_STATE_GOAL:
                    if task.turtle.wait_at_goal():
                        rospy.loginfo(task.turtle.name + ": successfully completed task")
                        task.free_turtle()
                        self.tasksAssigned.remove(task)

            for turtle in self.turtleBots:
                if turtle.docked == False and turtle.has_task == False:
                    turtle.go_to_waiting()
                    

            # send task status
            task_status = taskStatus()
            for a_task in self.tasksAssigned:
                t = task_msg()
                t.pose = a_task.position
                t.item = a_task.item
                t.id = a_task.id
                t.status = a_task.state
                t.num = a_task.taskID
                task_status.tasks.append(t)
            for a_task in self.tasks:
                t = task_msg()
                t.pose = a_task.position
                t.item = a_task.item
                t.id = a_task.id
                t.status = a_task.state
                t.num = a_task.taskID
                task_status.tasks.append(t)
            task_status_publisher.publish(task_status)
            rate.sleep()


if __name__ == '__main__':
    try:
        c = coordinator()
        c.main()
    except rospy.ROSInterruptException:
        pass
