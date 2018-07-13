#include "jackal_bsc/BSCTwistStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bsc_twist_stamped");
    ros::NodeHandle n;
    BSCTwistStamped bscts(&n);
    ROS_INFO("[TwistStamped]: Node ready.");
    ros::spin();
    return 0;
}
