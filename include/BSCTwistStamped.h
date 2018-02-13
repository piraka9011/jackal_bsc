#ifndef BSCTWISTSTAMPED_H
#define BSCTWISTSTAMPED_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class BSCTwistStamped
{
    private:
        ros::NodeHandle n;
        std::string nav_topic, nav_topic_stamped, key_topic, key_topic_stamped;
        ros::Publisher nav_stamped_pub;
        ros::Publisher key_stamped_pub;
        ros::Subscriber nav_sub;
        ros::Subscriber key_sub;

    public:
        BSCTwistStamped (ros::NodeHandle* nh);
        void nav_cb(const geometry_msgs::TwistConstPtr& nav_vel);
        void key_cb(const geometry_msgs::TwistConstPtr& key_vel);
};

#endif