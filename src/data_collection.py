#!/usr/bin/env python

import csv
import rospy
from std_msgs.msg import Float32


class DataCollection:
    def __init__(self):
        # Params
        user_name = rospy.get_param("/jackal_bsc/user_name", "Anas")
        self.drift_value = rospy.get_param("/jackal_bsc/drift_value", 1.0)
        self.delay_time = rospy.get_param("/jackal_bsc/delay_time", 1.0)
        exp_type = rospy.get_param("/jackal_bsc/exp_type", "BSC")
        # Subscribers
        rospy.Subscriber('/jackal_bsc/alpha', Float32, self.alpha_cb)
        # File name
        self.file_name = "../data/" + user_name + "_" + exp_type + ".csv"
        csv_file = open(self.file_name, 'wb')
        self.csv_writer = csv.writer(csv_file, delimiter=',')

    def alpha_cb(self, msg):
        self.csv_writer.writerow([msg, rospy.get_time()])

    def start(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Ending data collection")
            exit()


if __name__ == '__main__':
    d = DataCollection()
    d.start()
