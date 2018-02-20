
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

using namespace geometry_msgs;

ros::Publisher pub;
double drift_value;

void drift_cb(const TwistStampedConstPtr &delay_vel)
{
    TwistStamped new_vel;
    new_vel.header.stamp = ros::Time::now();
    new_vel.twist.angular.z = delay_vel->twist.angular.z + drift_value;
    new_vel.twist.linear.x = delay_vel->twist.linear.x;
    pub.publish(new_vel);
}

int main(int argc, char** argv)
{
    ros::NodeHandle n;
    std::string drift_topic, new_topic;
    n.param<std::string>("/jackal_bsc/drift_topic", drift_topic, "/jackal_bsc/key_vel_stamped");
    n.param<std::string>("/jackal_bsc/new_topic", new_topic, "/jackal_bsc/key_vel_drift");
    n.param<double>("/jackal_bsc/drift", drift_value, 1.0);
    pub = n.advertise<TwistStamped>(new_topic, 10);
    ros::Subscriber drift_sub = n.subscribe(drift_topic, 10, drift_cb);
    ros::spin();
    return 0;
}

