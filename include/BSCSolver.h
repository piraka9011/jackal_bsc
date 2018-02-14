//
// Created by river on 2/13/18.
//

#ifndef BSCSOLVER_H
#define BSCSOLVER_H

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

class BSCSolver
{

public:
    ros::NodeHandle n;
    std::string nav_topic, key_topic, bsc_topic;
    std::string odom_topic, goal_topic;
    double goal_x, goal_y, current_x, current_y;
    double delta_x, delta_y, dist_to_goal; // Displacement from current location to goal
    double delta_z, user_vel, navi_vel;
    double bsc_param;
    double max_dist;
    double max_vel;
    bool goal_received; // Check if action CB ran

    ros::Publisher bsc_pub;

    BSCSolver(ros::NodeHandle* nh);
    void bsc_cb(const geometry_msgs::TwistStampedConstPtr& nav_vel,
                const geometry_msgs::TwistStampedConstPtr& key_vel,
                const nav_msgs::OdometryConstPtr& odom);
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& goal_pose);
};


#endif //BSCSOLVER_H
