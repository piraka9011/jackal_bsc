#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>

using namespace geometry_msgs;

ros::Publisher pub;

void delay_cb(const geometry_msgs::TwistStampedConstPtr &delay_vel)
{
    ROS_INFO("Delay Callback");
    TwistStamped new_vel;
    new_vel.header.stamp = ros::Time::now();
    new_vel.twist.angular.z = delay_vel->twist.angular.z;
    new_vel.twist.linear.x = delay_vel->twist.linear.x;
    pub.publish(new_vel);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "delay_node");
    ros::NodeHandle n;
    // Params
    double delay_time;
    std::string delay_topic;
    std::string new_topic;

    n.param<std::string>("/jackal_bsc/delay_topic", delay_topic, "/jackal_bsc/key_vel_stamped");
    n.param<std::string>("/jackal_bsc/new_topic", new_topic, "/jackal_bsc/key_vel_delay");
    n.param("/jackal_bsc/delay_time", delay_time, 1.0);

    pub = n.advertise<TwistStamped>(new_topic, 10);

    // Time Sequencer
    message_filters::Subscriber<TwistStamped> sub(n, delay_topic, 1);
    message_filters::TimeSequencer<geometry_msgs::TwistStamped> seq(sub, ros::Duration(delay_time),
                                                     ros::Duration(0.01), 10);
    seq.registerCallback(delay_cb);

    ROS_INFO("Delay node created sequencer and is spinning...");
    ros::spin();
    return 0;
}

