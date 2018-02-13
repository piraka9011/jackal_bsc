//
// Created by river on 2/13/18.
//

#include "../include/BSCSolver.h"

BSCSolver::BSCSolver(ros::NodeHandle* nh):n(*nh)
{
    // Get params
    n.param<std::string>("nav_topic", nav_topic, "/bsc_jackal/nav_vel_stamped");
    n.param<std::string>("teleop_topic", key_topic, "/bsc_jackal/key_vel_stamped");
    n.param<std::string>("odom_topic", odom_topic, "/odometry/filtered");
    n.param<std::string>("goal_topic", goal_topic, "/move_base/current_goal");

    // Create normal subscribers
    goal_sub = n.subscribe(goal_sub, 10, &BSCSolver::goal_cb, this);

    // Create filter subscriber
    message_filters::Subscriber<TwistStamped> nav_sub(n, nav_topic, 1);
    message_filters::Subscriber<TwistStamped> key_sub(n, key_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, odom_topic, 1);

    // Create sync policy
    typedef sync_policies::ApproximateTime<TwistStamped, TwistStamped> AppxSyncPolicy;
    Synchronizer<AppxSyncPolicy> sync(AppxSyncPolicy(10), nav_sub, key_sub, odom_sub);
    sync.registerCallback(boost::bind(&bsc_cb, _1, _2, _3));
}

void BSCSolver::goal_cb(const geometry_msgs::PoseStampedConstPtr &goal_pose)
{
    goal_x = goal_pose->pose.position.x;
    goal_y = goal_pose->pose.position.y;
    goal_received = true;
}

void BSCSolver::bsc_cb(const geometry_msgs::TwistStampedConstPtr &nav_vel,
                       const geometry_msgs::TwistStampedConstPtr &key_vel)
{
    if (goal_received)
    {

    }
}
