#!/usr/bin/env python

import rospy
import roslaunch
from rospkg import RosPack
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from jackal_bsc.msg import Float64Stamped
import csv


class DataCollection:
    def __init__(self):
        # Subscribers
        rospy.Subscriber('/jackal_bsc/alpha', Float64Stamped, self.alpha_cb)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_cb)

        # Clients
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(3))

        # Params
        user_name = rospy.get_param("/jackal_bsc/name", "Anas")
        self.drift_value = rospy.get_param("/jackal_bsc/drift_value", 1.0)
        self.delay_time = rospy.get_param("/jackal_bsc/delay_time", 1.0)
        exp_type = rospy.get_param("/jackal_bsc/exp_type", "BSC")
        nav_type = rospy.get_param("/jackal_bsc/nav_type", "odom")
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
        self.goal.target_pose.header.frame_id = nav_type
        self.goal.target_pose.pose.position.x = -11.57
        self.goal.target_pose.pose.position.y = -15.32
        self.goal.target_pose.pose.orientation.z = 0.8785
        self.goal.target_pose.pose.orientation.w = -0.4775

        # Launch Bag file
        pkg = 'rosbag'
        type = 'record'
        bag_compression = '--lz4 '
        bag_name = '-O ' + pkg_path + '/data/' + user_name + '_' + exp_type + '.bag '
        topics = '/bluetooth_teleop/joy /cmd_vel /front/scan /imu/data /jackal_bsc/alpha ' \
                 '/jackal_bsc/bsc_vel /jackal_bsc/key_vel_stamped /jackal_bsc/nav_vel_stamped /jackal_laser_scan ' \
                 '/kinect2/qhd/camera_info /kinect2/qhd/image_color_rect ' \
                 '/kinect2/qhd/image_depth_rect /kinect2/qhd/points ' \
                 '/move_base/NavfnROS/plan /move_base/TrajectoryPlannerROS/cost_cloud ' \
                 '/move_base/TrajectoryPlannerROS/global_plan /move_base/TrajectoryPlannerROS/local_plan ' \
                 '/move_base/cancel /move_base/current_goal /move_base/feedback /move_base/global_costmap/costmap ' \
                 '/move_base/global_costmap/costmap_updates /move_base/global_costmap/footprint /move_base/goal' \
                 '/move_base/local_costmap/costmap /move_base/local_costmap/costmap_updates ' \
                 '/move_base/local_costmap/footprint ' \
                 '/move_base/local_costmap/obstacles_layer_footprint/footprint_stamped /move_base/result ' \
                 '/move_base/status /move_base_simple/goal /odometry/filtered /tf /tf_static '
        self.node = roslaunch.core.Node(pkg, type, args=bag_compression + bag_name + topics)
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        rospy.loginfo("\n[Data]: Name: {}\nExperiment Type: {}\n"
                      "Drift Value: {}\nDelay Value: {}".format(user_name, exp_type, self.drift_value, self.delay_time))

    def alpha_cb(self, msg):
        self.alpha_writer.writerow([msg.data, msg.header.stamp])

    def goal_cb(self, msg):
        if msg.status.status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[Data]: Goal reached, ending data collection.")
            self.goal_not_reached = False
            self.end()

    def start(self):
        rospy.loginfo("[Data]: Launching bag")
        self.roslaunch_process = self.launch.launch(self.node)
        rospy.loginfo("[Data]: Bagging: {}".format(self.roslaunch_process.is_alive()))
        try:
            self.move_client.send_goal(self.goal)
            self.start_time = rospy.Time().now()
            rospy.loginfo("[Data]: Start experiment")
            rospy.spin()
        except KeyboardInterrupt:
            rospy.logwarn("[Data]: User stopped! Ending data collection.")
            self.end()

    def end(self):
        rospy.loginfo("[Data]: Completing data collection...")
        rospy.signal_shutdown("End experiment")
        self.roslaunch_process.stop()
        self.end_time = rospy.Time().now()
        total_time = self.end_time - self.start_time
        format_time = str(total_time.secs) + '.' + str(total_time.nsecs)
        self.time_writer.writerow(["Start Time:", self.start_time])
        self.time_writer.writerow(["End Time:", self.end_time])
        self.time_writer.writerow(["Time:", format_time])
        self.time_writer.writerow(["Goal Reached:", self.goal_not_reached])
        rospy.loginfo("[Data]: Time: {}".format(format_time))
        rospy.loginfo("[Data]: Goal Not Reached: {}".format(self.goal_not_reached))
        self.alpha_csv.close()
        self.time_csv.close()
        exit(0)

    def __del__(self):
        self.end()


if __name__ == '__main__':
    rospy.init_node('data_collection')
    d = DataCollection()
    raw_input("Press enter to start the experiment...")
    d.start()
