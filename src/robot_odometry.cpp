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
    p_codom = n.advertise<ros_project_a::robotOdometry>("/car_odom", 50);
}

void RobotOdometry::broadcastTransform()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_dot, y_dot, 0));
    transform.setRotation(tf::createQuaternionFromRPY(0, 0, theta_dot));

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

void RobotOdometry::publishAsOdom(std::string base_link_name)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = std::move(base_link_name);

    odom.pose.pose.position.x = x_dot;
    odom.pose.pose.position.y = y_dot;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_dot);

    odom.twist.twist.linear.x = V_x;
    odom.twist.twist.linear.y = V_y;
    odom.twist.twist.angular.z = omega;

    p_odom.publish(odom);
}

double RobotOdometry::deg2rad(double degrees)
{
    return (degrees * M_PI) / 180;
}

void RobotOdometry::publishAsCustomMsg(std::string type)
{
    ros_project_a::robotOdometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";

    odom.x = x_dot;
    odom.y = y_dot;
    odom.theta = theta_dot;
    odom.type = std::move(type);

    p_codom.publish(odom);
}

