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

void RobotOdometry::publishAsOdom()
{
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "car";

    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    odom.twist.twist.linear.x = V;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = omega;

    p_odom.publish(odom);
}

void RobotOdometry::broadcastTransform()
{
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "car"));
}