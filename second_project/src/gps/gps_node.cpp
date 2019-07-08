#include "gpstransform.h"

int main(int argc, char **argv)
{
    // used to change the frame id from rimu to base_link
    ros::init(argc, argv, "gps_transform_node");

    GPSTransform gps_transform;

    ros::spin();
}
