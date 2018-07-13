#!/usr/bin/env python

import rospy
from rospkg import RosPack
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from std_msgs.msg import Float64
import csv


class DataCollection:
    def __init__(self):
        # Subscribers
        rospy.Subscriber('/jackal_bsc/alpha', Float64, self.alpha_cb)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_cb)

        # Clients
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(3))

        # Params
        user_name = rospy.get_param("/jackal_bsc/name", "Anas")
        self.drift_value = rospy.get_param("/jackal_bsc/drift_value", 1.0)
        self.delay_time = rospy.get_param("/jackal_bsc/delay_time", 1.0)
        exp_type = rospy.get_param("/jackal_bsc/exp_type", "BSC")
        self.start_time = 0
        self.end_time = 0
        self.goal_not_reached = True

        # File name
        rp = RosPack()
        pkg_path = rp.get_path('jackal_bsc')
        alpha_file = pkg_path + "/data/alpha_data/" + user_name + "_" + exp_type + "_" + "alpha.csv"
        time_file = pkg_path + "/data/time_data/" + user_name + "_" + exp_type + "_" + "time.csv"
        self.alpha_csv = open(alpha_file, 'wb')
        self.time_csv = open(time_file, 'wb')
        self.alpha_writer = csv.writer(self.alpha_csv, delimiter=',')
        self.time_writer = csv.writer(self.time_csv, delimiter=',')

        # Goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = 4.27
        self.goal.target_pose.pose.position.y = -8.55
        self.goal.target_pose.pose.orientation.z = 0.01
        self.goal.target_pose.pose.orientation.w = 0.999

    def alpha_cb(self, msg):
        self.alpha_writer.writerow([msg.data, rospy.get_time()])

    def goal_cb(self, msg):
        if msg.status.status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[Data]: Goal reached, ending data collection.")
            self.goal_not_reached = False
            self.end()

    def start(self):
        try:
            self.start_time = rospy.Time().now()
            self.move_client.send_goal(self.goal)
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("[Data]: User stopped! Ending data collection.")
            self.end()

    def end(self):
        rospy.loginfo("[Data]: Completing data collection...")
        self.end_time = rospy.Time().now()
        total_time = self.end_time - self.start_time
        self.time_writer.writerow([total_time])
        self.alpha_csv.close()
        self.time_csv.close()
        exit()


if __name__ == '__main__':
    rospy.init_node('data_collection')
    d = DataCollection()
    raw_input("Press enter to start the experiment...")
    d.start()
