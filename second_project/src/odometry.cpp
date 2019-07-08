#include <odometry.h>

Odometry::Odometry(double pos_x, double pos_y, double theta) : x_dot(pos_x), y_dot(pos_y), theta_dot(theta)
{
    time_ = ros::Time::now();
    speedsteer = n.subscribe("/speedsteer", 1000, &Odometry::calculate, this);

    p_odom = n.advertise<nav_msgs::Odometry>("/car/odometry/ackerman", 50);
}

void Odometry::broadcastTransform()
{
    geometry_msgs::TransformStamped odom_to_base;
    odom_to_base.header.frame_id = "odom";
    odom_to_base.header.stamp = ros::Time::now();
    odom_to_base.child_frame_id = "base_link";

    odom_to_base.transform.translation.x = x_dot;
    odom_to_base.transform.translation.y = y_dot;
    odom_to_base.transform.translation.z = 0;
    odom_to_base.transform.rotation = tf::createQuaternionMsgFromYaw(theta_dot);

    broadcaster.sendTransform(odom_to_base);
}

void Odometry::publishAsOdom()
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_dot;
    odom.pose.pose.position.y = y_dot;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_dot);

    odom.twist.twist.linear.x = V_x;
    odom.twist.twist.linear.y = V_y;
    odom.twist.twist.angular.z = omega;

    p_odom.publish(odom);
}

inline double Odometry::deg2rad(double degrees)
{
    return (degrees * M_PI) / 180;
}

inline double Odometry::kmph2mps(double speed_km_per_hour){
    return speed_km_per_hour / 3.6;
}

void Odometry::calculate(const geometry_msgs::PointStampedConstPtr &speed_steer)
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

    std::cout <<  alpha << std::endl<< dt << std::endl<< V << std::endl<< omega << std::endl<< theta_dot << std::endl;


    this->broadcastTransform();
    this->publishAsOdom();
}
