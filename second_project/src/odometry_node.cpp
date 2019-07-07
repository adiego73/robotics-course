#include <ackerman_odom.h>

int main(int argc, char **argv)
{
    double pos_x = 0.0;
    double pos_y = 0.0;
    double theta = 0.0;

    if(argc >= 4){
        pos_x = std::strtod(argv[1], NULL);
        pos_y = std::strtod(argv[2], NULL);
        theta = std::strtod(argv[3], NULL);
    }

    ros::init(argc, argv, "odometry_node");

    AckermanOdometry ak_odometry(pos_x, pos_y, theta);

    ros::spin();
}
