#include "imutransform.h"

int main(int argc, char **argv)
{
    // used to change the frame id from rimu to base_link
    ros::init(argc, argv, "imu_transform_node");

    ImuTransform imu_transform;

    ros::spin();
}
