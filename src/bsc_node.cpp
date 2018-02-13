#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

using namespace geometry_msgs;
using namespace message_filters;

void bsc_cb(const TwistStampedConstPtr& nav_vel,
            const TwistStampedConstPtr& key_vel)
{
    ROS_INFO("Entered Callback");
    ROS_INFO("nav_vel: %f, key_vel: %f",
             nav_vel->twist.linear.x, key_vel->twist.linear.x);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bsc_node");

    ros::NodeHandle n;

    // Get Params
    std::string nav_topic, key_topic, bsc_topic;
    std::string odom_topic;
    n.param<std::string>("nav_topic", nav_topic, "/bsc_jackal/nav_vel_stamped");
    n.param<std::string>("teleop_topic", key_topic, "/bsc_jackal/key_vel_stamped");
    n.param<std::string>("odom_topic", odom_topic, "/odometry/filtered");
    ROS_INFO("Navigation Topic: %s\n Teleop Topic:%s\n",
             nav_topic.c_str(), key_topic.c_str());
    // Create filter subscriber
    message_filters::Subscriber<TwistStamped> nav_sub(n, nav_topic, 1);
    message_filters::Subscriber<TwistStamped> key_sub(n, key_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, odom_topic, 1);

    // Create sync policy
    typedef sync_policies::ApproximateTime<TwistStamped, TwistStamped> AppxSyncPolicy;
    Synchronizer<AppxSyncPolicy> sync(AppxSyncPolicy(10), nav_sub, key_sub, odom_sub);
    sync.registerCallback(boost::bind(&bsc_cb, _1, _2, _3));

    ROS_INFO("Spinning");
    ros::spin();

    return 0;
}