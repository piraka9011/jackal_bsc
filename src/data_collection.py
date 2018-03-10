#!/usr/bin/env python

import functools
import rospy
from geometry_msgs.msg import TwistStamped


class DataCollection:
    def __init__(self):
        user_name = rospy.get_param("/jackal_bsc/user_name", "Anas")
        self.drift_value = rospy.get_param("/jackal_bsc/drift_value", 1.0)
        self.delay_time = rospy.get_param("/jackal_bsc/delay_time", 1.0)
        exp_type = rospy.get_param("/jackal_bsc/exp_type", "BSC")
        self.file_name = "../data/" + user_name + "_" + exp_type + ".csv"

    