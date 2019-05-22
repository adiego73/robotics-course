#include <differential.h>

Differential::Differential(double pos_x, double pos_y, double theta) : RobotOdometry(pos_x, pos_y, theta)
{

    sync.registerCallback(boost::bind(&Differential::calculate, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom/differential", 50);
}

void Differential::calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer)
{
    const ros::Time& current_time = V_r->header.stamp;
    double dt = (current_time - time_).toSec();
    time_ = current_time;

    V = (V_r->data + V_l->data) / 2.0;
    omega = (V_r->data - V_l->data) / REAR_WHEELS_BASE_LINE;

    pos_x = pos_x + V * std::cos(omega) * dt;
    pos_y = pos_y + V * std::sin(omega) * dt;
    theta = theta + (omega * dt);

    V_x = V * std::cos(theta);
    V_y = V * std::sin(theta);

    tf::Quaternion q;
    q.setRPY(0, 0, this->theta);

    transform.setOrigin(tf::Vector3(this->pos_x, this->pos_y, 0));
    transform.setRotation(q);

#ifdef DEBUG
    ROS_INFO("Differential Drive :: position in X: %f", pos_x);
    ROS_INFO("Differential Drive :: position in Y: %f", pos_y);
    ROS_INFO("Differential Drive :: steering angle: %f", theta);
#endif

    if (this->active) {
        this->broadcastTransform();
        this->publishAsOdom();
    }

}

