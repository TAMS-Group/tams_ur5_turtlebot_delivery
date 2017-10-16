#! /usr/bin/env python
__author__ = '0rokita'

import rospy

from project15_turtlebot.msg import OnlineTurtle
import os
class TurtleAnnounce(object):
    def __init__(self):
        rospy.init_node('turtle_announce', anonymous=True)
        self.name = os.environ['TURTLEBOT_NAME']

    def publish(self):
        pub = rospy.Publisher('turtles', OnlineTurtle, queue_size=10)
        rate = rospy.Rate(1)

        turtle = OnlineTurtle(name= self.name)

        while not rospy.is_shutdown():
            try:
                turtle.header.stamp = rospy.Time.now()
                pub.publish(turtle)
            except Exception as e:
                print e
            rate.sleep()


if __name__ == '__main__':
    try:
        o = TurtleAnnounce()
        o.publish()
    except rospy.ROSInterruptException:
        pass

