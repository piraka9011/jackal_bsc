//
// Created by river on 2/13/18.
//

#include "jackal_bsc/BSCSolver.h"

using namespace geometry_msgs;
using namespace message_filters;

BSCSolver::BSCSolver(ros::NodeHandle* nh):n(*nh)
{
    // Get params
    n.param<std::string>("nav_topic_stamped", nav_topic, "/jackal_bsc/nav_vel_stamped");
    n.param<std::string>("teleop_topic_stamped", key_topic, "/jackal_bsc/key_vel_stamped");
    n.param<std::string>("bsc_topic", bsc_topic, "/jackal_bsc/bsc_vel");
    n.param<std::string>("odom_topic", odom_topic, "/jackal_velocity_controller/odom");
    n.param<std::string>("goal_topic", goal_topic, "/move_base/current_goal");
    max_dist = 15.0;
    max_vel = 3.0;
    goal_received = false;
    // Create publishers
    bsc_pub = n.advertise<TwistStamped>("/bsc_jackal/bsc_vel", 10);

    // Create normal subscribers
    ros::Subscriber goal_sub = n.subscribe(goal_topic, 10, &BSCSolver::goal_cb, this);

    // Create filter subscriber
    message_filters::Subscriber<TwistStamped> nav_sub(n, nav_topic, 1);
    message_filters::Subscriber<TwistStamped> key_sub(n, key_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, odom_topic, 1);

    // Create sync policy
    typedef sync_policies::ApproximateTime<TwistStamped, TwistStamped, nav_msgs::Odometry> AppxSyncPolicy;
    Synchronizer<AppxSyncPolicy> sync(AppxSyncPolicy(10), nav_sub, key_sub, odom_sub);
    // Register BSC callback to get all three topics
    sync.registerCallback(boost::bind(&BSCSolver::bsc_cb, this, _1, _2, _3));
}


void BSCSolver::goal_cb(const PoseStampedConstPtr &goal_pose)
{
    goal_x = goal_pose->pose.position.x;
    goal_y = goal_pose->pose.position.y;
    goal_received = true;
}

void BSCSolver::bsc_cb(const geometry_msgs::TwistStampedConstPtr &nav_vel,
                       const geometry_msgs::TwistStampedConstPtr &key_vel,
                       const nav_msgs::OdometryConstPtr& odom)
{
    if (goal_received)
    {
        // Compute difference in user and navigation commands
        user_vel_z = key_vel->twist.angular.z;
        user_vel_x = key_vel->twist.angular.x;
        navi_vel = nav_vel->twist.angular.z;
        delta_z = user_vel_z - navi_vel;

        // Compute displacement from current position to goal
        current_x = odom->pose.pose.position.x;
        current_y = odom->pose.pose.position.y;
        delta_x = goal_x - current_x;
        delta_y = goal_y - current_y;
        dist_to_goal = hypot(delta_x, delta_y);

        // Compute BSC parameter
        bsc_param = fmax(0, (1 - (dist_to_goal/max_dist))) *
                    fmax(0, (1 - pow(delta_z / max_vel, 2)));
        // Apply blending
        geometry_msgs::TwistStamped blended_vel;
        blended_vel.header.stamp = ros::Time::now();
        blended_vel.twist.angular.z = user_vel_z - (bsc_param * delta_z);
        blended_vel.twist.linear.x = user_vel_x;
        bsc_pub.publish(blended_vel);
    }
}
