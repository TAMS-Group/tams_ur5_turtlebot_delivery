#! /usr/bin/env python
__author__ = '0rokita'

import rospy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import tf

class MapPosition(object):

    def __init__(self):
        rospy.init_node('map_position', anonymous=True)
        self.tf = tf.TransformListener()

    def subscribe(self):

        pub = rospy.Publisher('map_position', PoseStamped, queue_size=10)

        rate = rospy.Rate(10) # 10hz

        pos = PoseStamped(header= Header(frame_id="map"))

        while not rospy.is_shutdown():
            try:
                time = self.tf.getLatestCommonTime("map", "base_footprint")
                trans, rot = self.tf.lookupTransform("map", "base_footprint", time)

                pos.header.stamp = time
                pos.pose.position = Point(*trans)
                pos.pose.orientation = Quaternion(*rot)

                pub.publish(pos)
            except Exception as e:
                print e
            rate.sleep()


if __name__ == '__main__':
    try:
        o = MapPosition()
        o.subscribe()
    except rospy.ROSInterruptException:
        pass

