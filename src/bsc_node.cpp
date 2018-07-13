#include "jackal_bsc/BSCSolver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bsc_node");
    ros::NodeHandle n;
    BSCSolver bscs(&n);
    ROS_INFO("[BSC]: Node ready");
    ros::spin();
    return 0;
}