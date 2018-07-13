#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped

if __name__ == '__main__':
    rospy.init_node('delay_test')
    pub = rospy.Publisher('/jackal_bsc/key_vel_stamped', TwistStamped, queue_size=10)
    while pub.get_num_connections() == 0: rospy.sleep(0.1)
    t = TwistStamped()
    t.header.stamp = rospy.Time.now()
    rospy.loginfo("Publishing: {}".format(t))
    pub.publish(t)
