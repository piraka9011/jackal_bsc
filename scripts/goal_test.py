#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

def goal_cb(msg):
    if msg.status.status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Result SUCCEEDED")

if __name__ == '__main__':
    rospy.init_node('goal_test')
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_cb)
    cli = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Waiting...")
    cli.wait_for_server(rospy.Duration(3))
    rospy.loginfo("Got Server")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time().now()
    goal.target_pose.pose.position.x = 0.4
    goal.target_pose.pose.position.y = 0.685
    goal.target_pose.pose.orientation.z = 0.37
    goal.target_pose.pose.orientation.w = 0.929
    cli.send_goal(goal)
    # rospy.loginfo("Waiting for result...")
    # cli.wait_for_result()
    # state = cli.get_state()
    # result = cli.get_result()
    # rospy.loginfo("State: {}\n Result: {}\n".format(state, result))
    # if state == actionlib.GoalStatus.SUCCEEDED:
    #     rospy.loginfo("State DONE")
    rospy.spin()