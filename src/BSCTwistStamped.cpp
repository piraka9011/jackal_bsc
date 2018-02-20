#include "jackal_bsc/BSCTwistStamped.h"


BSCTwistStamped::BSCTwistStamped(ros::NodeHandle* nh):n(*nh)
{
    // Get topic names
    n.param<std::string>("/jackal_bsc/nav_topic", nav_topic, "/jackal_bsc/nav_vel");
    n.param<std::string>("/jackal_bsc/teleop_topic", key_topic, "/jackal_bsc/key_vel");
    n.param<std::string>("/jackal_bsc/nav_topic_stamped", nav_topic_stamped, "/jackal_bsc/nav_vel_stamped");
    n.param<std::string>("/jackal_bsc/teleop_topic_stamped", key_topic_stamped, "/jackal_bsc/key_vel_stamped");
    ROS_INFO("Navigation Topic: %s\n Stamped Navigation Topic: %s\n"
             "Teleop Topic: %s\n Stamped Teleop Topic: %s\n",
              nav_topic.c_str(), nav_topic_stamped.c_str(), key_topic.c_str(), key_topic_stamped.c_str());
    // Create pubs/subs
    nav_stamped_pub = n.advertise<geometry_msgs::TwistStamped>(nav_topic_stamped, 10);
    key_stamped_pub = n.advertise<geometry_msgs::TwistStamped>(key_topic_stamped, 10);
    nav_sub = n.subscribe(nav_topic, 10, &BSCTwistStamped::nav_cb, this);
    key_sub = n.subscribe(key_topic, 10 , &BSCTwistStamped::key_cb, this);
}

/**
 * Convert incoming Twist message into a TwistStamped
 * @param nav_vel Incoming Twist message
 */
void BSCTwistStamped::nav_cb(const geometry_msgs::TwistConstPtr& nav_vel)
{
    geometry_msgs::TwistStamped new_twist;
    new_twist.header.stamp = ros::Time::now();
    new_twist.twist.linear.x = nav_vel->linear.x;
    new_twist.twist.angular.z = nav_vel->angular.z;
    nav_stamped_pub.publish(new_twist);
}

void BSCTwistStamped::key_cb(const geometry_msgs::TwistConstPtr& key_vel)
{
    geometry_msgs::TwistStamped new_twist;
    new_twist.header.stamp = ros::Time::now();
    new_twist.twist.linear.x = key_vel->linear.x;
    new_twist.twist.angular.z = key_vel->angular.z;
    key_stamped_pub.publish(new_twist);
}
