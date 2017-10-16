
import rospy 

TASK_STATE_INITIAL = 0
TASK_STATE_FETCH_TURTLE = 1
TASK_STATE_GET_ITEM = 2
TASK_STATE_GOAL = 3
TASK_STATE_DELIVER = 4
TASK_STATE_POSITION_TURTLE = 5

class Task(object):
    ID_counter = 1

    def __init__(self, position, item, id):
        self.taskID = Task.ID_counter
        self.item = item
        Task.ID_counter = Task.ID_counter + 1
        self.position = position
        self.state = TASK_STATE_INITIAL
        # the used Turtle
        self.turtle = None
        self.id = id

    def getID(self):
        return self.taskID

    def getGoal(self):
        return self.position

    def assign_turtle(self, turtle):
        rospy.loginfo("Assign turtle " + turtle.name + " to task " + str(self))
        self.turtle = turtle
        self.turtle.has_task = True

    def free_turtle(self):
        self.turtle.has_task = False

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "<Task: ID=%d/%s Item=%s>" % (self.taskID, self.id, self.item)
