#include "jackal_bsc/BSCTwistStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bsc_twist_stamped");
    ros::NodeHandle n;
    BSCTwistStamped bscts(&n);
    ROS_INFO("TwistStamped publisher node created");
    ROS_INFO("Spinning...");
    ros::spin();
    return 0;
}
