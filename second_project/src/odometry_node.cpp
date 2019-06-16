#include <ackerman_odom.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");

    AckermanOdometry ak_odometry(0, 0, 0);

    ros::spin();
}
