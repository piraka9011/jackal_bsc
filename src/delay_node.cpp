
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>

using namespace geometry_msgs;
using namespace message_filters;

ros::Publisher pub;

void delay_cb(const TwistStampedConstPtr& delay_vel)
{
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
    double delay_time = 1.0;
    std::string delay_topic = "/jackal_bsc/key_vel_stamped";
    std::string new_topic = "/jackal_bsc/key_vel_delay";
    n.param<std::string>("/jackal_bsc/delay_topic", delay_topic, "/jackal_bsc/key_vel_stamped");
    n.param<std::string>("/jackal_bsc/new_topic", new_topic, "/jackal_bsc/key_vel_delay");
    n.param("/jackal_bsc/delay_time", delay_time, 1.0);
    pub = n.advertise<TwistStamped>(new_topic, 10);
    message_filters::Subscriber<TwistStamped> sub(n, delay_topic, 1);
    message_filters::TimeSequencer<TwistStamped> seq(sub, ros::Duration(0.1), ros::Duration(0.01), 10);
    seq.registerCallback(delay_cb);
    ros::spin();
    return 0;
}

