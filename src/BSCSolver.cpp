#include "jackal_bsc/BSCSolver.h"

/**
 * Stuff learned:
 * 1. Subscribers were defined local to constructor, meaning callback never executed.
 * Defining the subscribers as class methods and dynamically initializing them in constructor
 * works.
 * https://answers.ros.org/question/60903/timesynchronizer-callback-problem/
 * 2. To use callback in class, you need to pass reference to this
 */

using namespace geometry_msgs;
using namespace message_filters;

BSCSolver::BSCSolver(ros::NodeHandle* nh):
    n(*nh)
{
    // Get params
    n.param<std::string>("/jackal_bsc/bsc_nav_topic", nav_topic, "/jackal_bsc/nav_vel_stamped");
    n.param<std::string>("/jackal_bsc/bsc_teleop_topic", key_topic, "/jackal_bsc/key_vel_stamped");
    n.param<std::string>("/jackal_bsc/bsc_topic", bsc_topic, "/jackal_bsc/bsc_vel");
    n.param<std::string>("/jackal_bsc/odom_topic", odom_topic, "/jackal_velocity_controller/odom");
    n.param<std::string>("/jackal_bsc/goal_topic", goal_topic, "/move_base/current_goal");
    max_dist = 15.0;
    max_vel = 3.0;
    q_size = 10;
    seq = 0;
    goal_received = false;
    // Create publishers
    bsc_pub = n.advertise<Twist>("/jackal_bsc/bsc_vel", q_size);
    alpha_pub = n.advertise<jackal_bsc::Float64Stamped>("/jackal_bsc/alpha", q_size);

    // Create normal subscribers
    goal_sub = n.subscribe(goal_topic, 10, &BSCSolver::goal_cb, this);

    // Create filter subscriber
    nav_sub = new twist_sub_type(n, nav_topic, q_size);
    key_sub = new twist_sub_type(n, key_topic, q_size);
    odom_sub = new odom_sub_type(n, odom_topic, q_size);

    // Create sync policy
    appx_sync = new message_filters::Synchronizer<AppxSyncPolicy>(AppxSyncPolicy(q_size),
                                                                  *nav_sub, *key_sub, *odom_sub);
    // Register BSC callback to get all three topics
    appx_sync->registerCallback(boost::bind(&BSCSolver::bsc_cb, this, _1, _2, _3));
}

void BSCSolver::goal_cb(const PoseStampedConstPtr &goal_pose)
{
    goal_x = goal_pose->pose.position.x;
    goal_y = goal_pose->pose.position.y;
    goal_received = true;
    ROS_INFO("[BSC]: Node received a goal!");
}

void BSCSolver::bsc_cb(const geometry_msgs::TwistStampedConstPtr &nav_vel,
                       const geometry_msgs::TwistStampedConstPtr &key_vel,
                       const nav_msgs::OdometryConstPtr& odom)
{
    ROS_WARN_THROTTLE(15, "[BSC]: No goal received...");
    if (goal_received) {
        ROS_INFO_THROTTLE(10, "[BSC]: Blending...");
        seq++;
        // Compute difference in user and navigation commands
        user_vel_z = key_vel->twist.angular.z;
        user_vel_x = key_vel->twist.linear.x;
        navi_vel = nav_vel->twist.angular.z;
        delta_z = user_vel_z - navi_vel;

        // Compute displacement from current position to goal
        current_x = odom->pose.pose.position.x;
        current_y = odom->pose.pose.position.y;
        delta_x = goal_x - current_x;
        delta_y = goal_y - current_y;
        dist_to_goal = hypot(delta_x, delta_y);

        // Compute BSC parameter
        bsc_param.header.seq = seq;
        bsc_param.header.stamp = ros::Time::now();
        bsc_param.data = fmax(0, (1 - (dist_to_goal / max_dist))) *
                    fmax(0, (1 - pow(delta_z / max_vel, 2)));
        alpha_pub.publish(bsc_param);

        // Apply blending
        geometry_msgs::Twist blended_vel;
        blended_vel.angular.z = user_vel_z - (bsc_param.data * delta_z);
        blended_vel.linear.x = user_vel_x;
        bsc_pub.publish(blended_vel);
    }
}
