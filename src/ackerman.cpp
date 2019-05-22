#include <ackerman.h>

Ackerman::Ackerman(double pos_x, double pos_y, double theta) : RobotOdometry(pos_x, pos_y, theta)
{

    sync.registerCallback(boost::bind(&Ackerman::calculate, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom/ackerman", 50);
}

void Ackerman::calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer)
{

    double alpha = steer->data / STEERING_FACTOR;

    const ros::Time& current_time = V_r->header.stamp;
    double dt = (current_time - time_).toSec();
    time_ = current_time;

    V = (V_r->data + V_l->data) / 2.0;
    omega = V * std::tan(alpha) / FRONT_REAR_DISTANCE;

    pos_x = pos_x + V * dt * std::cos(theta + (omega * dt / 2));
    pos_y = pos_y + V * dt * std::sin(theta + (omega * dt / 2));
    theta = theta + (omega * dt);

    V_x = V * std::cos(alpha);
    V_y = V * std::sin(alpha);

    tf::Quaternion q;
    q.setRPY(0, 0, this->theta);

    transform.setOrigin(tf::Vector3(this->pos_x, this->pos_y, 0));
    transform.setRotation(q);

#ifdef DEBUG
    ROS_INFO("Ackerman :: position in X: %f", pos_x);
    ROS_INFO("Ackerman :: position in Y: %f", pos_y);
    ROS_INFO("Ackerman :: steering alpha: %f", theta);
#endif

    if (this->active) {
        this->broadcastTransform();
        this->publishAsOdom();
    }
}
