#include <ackerman.h>

Ackerman::Ackerman(double pos_x, double pos_y, double theta) : RobotOdometry(pos_x, pos_y, theta)
{

    sync.registerCallback(boost::bind(&Ackerman::calculate, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom/ackerman", 50);
}

void Ackerman::calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer)
{
    double alpha = this->deg2rad(steer->data) / STEERING_FACTOR;

    const ros::Time& current_time = V_r->header.stamp;
    double dt = (current_time - time_).toSec();
    time_ = current_time;

    V = (V_r->data + V_l->data) / 2.0;
    omega = V * std::tan(alpha) / FRONT_REAR_DISTANCE;

    V_x = V * std::cos(theta_dot);
    V_y = V * std::sin(theta_dot);

    x_dot += V * std::cos(theta_dot) * dt;
    y_dot += V * std::sin(theta_dot) * dt;
    theta_dot += omega * dt;

#ifdef DEBUG
    ROS_INFO("Ackerman :: position in X: %f", x_dot);
    ROS_INFO("Ackerman :: position in Y: %f", y_dot);
    ROS_INFO("Ackerman :: steering alpha: %f", theta_dot);
#endif

    if (this->active) {
        this->broadcastTransform();
        this->publishAsOdom("base_link_d");
    }
}
