#ifndef BSCSOLVER_H
#define BSCSOLVER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <math.h>
#include <iostream>
#include <fstream>

typedef message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub_type;
typedef message_filters::Subscriber<nav_msgs::Odometry> odom_sub_type;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped,
        geometry_msgs::TwistStamped, nav_msgs::Odometry> AppxSyncPolicy;

class BSCSolver
{

public:
    ros::NodeHandle n;
    std::string nav_topic, key_topic, bsc_topic;
    std::string odom_topic, goal_topic;
    std::string exp_type, user_name;
    double goal_x, goal_y, current_x, current_y;
    double delta_x, delta_y, dist_to_goal; // Displacement from current location to goal
    double delta_z, user_vel_z, navi_vel;
    double user_vel_x, user_delay_z, user_delay_x;
    std_msgs::Float64 bsc_param;
    double max_dist;
    double max_vel;
    int q_size;
    bool goal_received; // Check if action CB ran

    ros::Publisher bsc_pub;
    ros::Publisher alpha_pub;
    ros::Subscriber goal_sub;

    message_filters::Subscriber<geometry_msgs::TwistStamped>* nav_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped>* key_sub;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub;
    message_filters::Synchronizer<AppxSyncPolicy>* appx_sync;

    BSCSolver(ros::NodeHandle* nh);
    void bsc_cb(const geometry_msgs::TwistStampedConstPtr& nav_vel,
                const geometry_msgs::TwistStampedConstPtr& key_vel,
                const nav_msgs::OdometryConstPtr& odom);
    void goal_cb(const geometry_msgs::PoseStampedConstPtr& goal_pose);
};


#endif //BSCSOLVER_H
