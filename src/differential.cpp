#include <differential.h>

Differential::Differential(double pos_x, double pos_y, double theta) : RobotOdometry(pos_x, pos_y, theta)
{

    sync.registerCallback(boost::bind(&Differential::calculate, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom/differential", 50);
}

void Differential::calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer)
{
//    const ros::Time& current_time = V_r->header.stamp;
//    double dt = (current_time - time_).toSec();
//    time_ = current_time;

    double dt = 1;

    V = (V_r->data + V_l->data) / 2.0;
    omega = (V_r->data - V_l->data) / REAR_WHEELS_BASE_LINE;

    V_x = V * std::cos(theta_dot);
    V_y = V * std::sin(theta_dot);

    x_dot += (V_x * std::cos(theta_dot) - V_y * std::sin(theta_dot)) * dt;
    y_dot += (V_x * std::sin(theta_dot) + V_y * std::cos(theta_dot)) * dt;
    theta_dot += omega * dt;

#ifdef DEBUG
    ROS_INFO("Differential Drive :: position in X: %f", x_dot);
    ROS_INFO("Differential Drive :: position in Y: %f", y_dot);
    ROS_INFO("Differential Drive :: steering angle: %f", theta_dot);
#endif

    if (this->active) {
        this->broadcastTransform();
        this->publishAsOdom("base_link_d");
    }

}

