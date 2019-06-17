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

    V_x = V * std::cos(theta_dot);
    V_y = V * std::sin(theta_dot);

    x_dot += V * std::cos(theta_dot) * dt;
    y_dot += V * std::sin(theta_dot) * dt;

//    // integration of pos_y
//    if (omega <= 0.2) {
//        // 2nd Runge-Kutta
//        y_dot += V * timeDiff * (std::sin(theta + (omega * timeDiff / 2)));
//        x_dot += V * timeDiff * (std::cos(theta + (omega * timeDiff / 2)));
//    } else {
//        // exact integration of x and y
//        y_dot -= (V / omega) * (std::cos(omega) - std::cos(theta));
//        x_dot += (V / omega) * (std::sin(omega) - std::sin(theta));
//    }

    theta_dot += (omega * timeDiff);


#ifdef DEBUG
    ROS_INFO("Differential Drive :: position in X: %f", x_dot);
    ROS_INFO("Differential Drive :: position in Y: %f", y_dot);
    ROS_INFO("Differential Drive :: steering angle: %f", theta_dot);
#endif

    if (this->active) {
        this->broadcastTransform();
        this->publishAsOdom("base_link_d");
        this->publishAsCustomMsg("Differential");
    }

}

