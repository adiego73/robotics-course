#include <robot_odometry.h>

void RobotOdometry::activate(bool flag)
{
    this->active = flag;
}

void RobotOdometry::setPositionX(double x)
{
    this->x_dot = x;
}

void RobotOdometry::setPostionY(double y)
{
    this->y_dot = y;
}

RobotOdometry::RobotOdometry(double pos_x, double pos_y, double theta)
        : x_dot(pos_x), y_dot(pos_y), theta_dot(theta), speed_r(n, "/speedR_stamped", 1),
          speed_l(n, "/speedL_stamped", 1), steer(n, "/steer_stamped", 1), sync(SyncPolicy(10), speed_r, speed_l, steer)
{

}

void RobotOdometry::publishAsOdom(nav_msgs::Odometry odom)
{
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";

    p_odom.publish(odom);
}

void RobotOdometry::broadcastTransform()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_dot, y_dot, 0));
    transform.setRotation(tf::createQuaternionFromRPY(0, 0, theta_dot));

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}