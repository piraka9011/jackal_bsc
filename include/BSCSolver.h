//
// Created by river on 2/13/18.
//

#ifndef BSCSOLVER_H
#define BSCSOLVER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>

class BSCSolver {

private:
    ros::NodeHandle n;
    std::string nav_topic, key_topic, bsc_topic;
    std::string odom_topic, goal_topic;
    float goal_x, goal_y;
    bool goal_received = false;

    void bsc_cb(const geometry_msgs::TwistStampedConstPtr& nav_vel,
                const geometry_msgs::TwistStampedConstPtr& key_vel);
    void goal_cb(const geometry_msgs::PoseStamped& goal_pose);
};


#endif //BSCSOLVER_H
