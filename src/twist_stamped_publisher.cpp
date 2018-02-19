//
// Created by river on 2/13/18.
//
#include "jackal_bsc/BSCTwistStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bsc_twist_stamped");
    ros::NodeHandle n;

    BSCTwistStamped bscts(&n);

    ros::spin();
    return 0;
}
