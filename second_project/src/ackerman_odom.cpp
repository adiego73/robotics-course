#include "ackerman_odom.h"

AckermanOdometry::AckermanOdometry(double pos_x, double pos_y, double theta) : x_dot(pos_x), y_dot(pos_y), theta_dot(theta)
{
    speedsteer = n.subscribe("/speedsteer", 1000, &AckermanOdometry::calculate, this);

    p_odom = n.advertise<nav_msgs::Odometry>("/car/odometry/ackerman", 50);
}

void AckermanOdometry::broadcastTransform()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_dot, y_dot, 0));
    transform.setRotation(tf::createQuaternionFromRPY(0, 0, theta_dot));

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), FRAME_ID, CHILD_FRAME_ID));
}

void AckermanOdometry::publishAsOdom()
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = FRAME_ID;
    odom.child_frame_id = CHILD_FRAME_ID;

    odom.pose.pose.position.x = x_dot;
    odom.pose.pose.position.y = y_dot;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_dot);

    odom.twist.twist.linear.x = V_x;
    odom.twist.twist.linear.y = V_y;
    odom.twist.twist.angular.z = omega;

    p_odom.publish(odom);
}

double AckermanOdometry::deg2rad(double degrees)
{
    return (degrees * M_PI) / 180;
}

double AckermanOdometry::kmph2mps(double speed_km_per_hour){
    return speed_km_per_hour / 3.6;
}

void AckermanOdometry::calculate(const geometry_msgs::PointStampedConstPtr &speed_steer)
{
    double alpha = this->deg2rad(speed_steer->point.x) / STEERING_FACTOR;

    const ros::Time& current_time = speed_steer->header.stamp;
    double dt = (current_time - time_).toSec();
    time_ = current_time;

    V = this->kmph2mps(speed_steer->point.y);
    omega = V * std::tan(alpha) / FRONT_REAR_DISTANCE;

    V_x = V * std::cos(theta_dot);
    V_y = V * std::sin(theta_dot);

    x_dot += V * std::cos(theta_dot) * dt;
    y_dot += V * std::sin(theta_dot) * dt;
    theta_dot += omega * dt;

    this->broadcastTransform();
    this->publishAsOdom();
}
