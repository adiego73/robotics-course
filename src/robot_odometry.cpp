#include <robot_odometry.h>

void RobotOdometry::activate(bool flag)
{
    this->active = flag;
}

void RobotOdometry::setPositionX(double x)
{
    this->pos_x = x;
}

void RobotOdometry::setPostionY(double y)
{
    this->pos_y = y;
}

RobotOdometry::RobotOdometry(double pos_x, double pos_y, double theta)
        : pos_x(pos_x), pos_y(pos_y), theta(theta), speed_r(n, "/speedR_stamped", 1), speed_l(n, "/speedL_stamped", 1),
          steer(n, "/steer_stamped", 1), sync(SyncPolicy(10), speed_r, speed_l, steer)
{

}
