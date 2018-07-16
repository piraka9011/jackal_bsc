#!/usr/bin/env python

import functools
import rospy
from geometry_msgs.msg import TwistStamped, Twist


class DelayNode:
    def __init__(self):
        self.delay_time = rospy.get_param("/jackal_bsc/delay_time", 1.0)
        delay_topic = rospy.get_param("/jackal_bsc/delay_topic", "/jackal_bsc/key_vel_stamped")
        new_topic = rospy.get_param("/jackal_bsc/new_topic", "/jackal_bsc/key_vel_delay")

        self.delay_pub = rospy.Publisher(new_topic, TwistStamped, queue_size=10)
        sub = rospy.Subscriber(delay_topic, TwistStamped, self.callback, queue_size=40)

        # For testing
        # delay_topic = '/jackal_bsc/key_vel'
        # new_topic = '/jackal_bsc/key_vel_delay'
        # self.delay_pub = rospy.Publisher(new_topic, Twist, queue_size=10)
        # sub = rospy.Subscriber(delay_topic, Twist, self.callback, queue_size=40)

    def delayed_callback(self, msg, event):
        # Change header time to sync with nav_vel
        msg.header.stamp = rospy.Time().now()
        self.delay_pub.publish(msg)

    def callback(self, msg):
        timer = rospy.Timer(rospy.Duration(self.delay_time),
                            functools.partial(self.delayed_callback, msg),
                            oneshot=True)

    def start(self):
        rospy.loginfo("[Delay]: Delay node started")
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("delay_node")
    d = DelayNode()
    d.start()
