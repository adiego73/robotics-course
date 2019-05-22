#include <ackerman.h>

Ackerman::Ackerman(double pos_x, double pos_y, double theta) : RobotOdometry(pos_x, pos_y, theta)
{

    sync.registerCallback(boost::bind(&Ackerman::calculate, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom/ackerman", 50);
}

void Ackerman::calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer)
{

    double alpha = steer->data / STEERING_FACTOR;

//    const ros::Time& current_time = V_r->header.stamp;
//    double dt = (current_time - time_).toSec();
//    time_ = current_time;

    double dt = 1;

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
        this->publishAsOdom();
    }
}

void Ackerman::publishAsOdom()
{
    nav_msgs::Odometry odom;
    odom.child_frame_id = "base_link_a";

    odom.pose.pose.position.x = x_dot;
    odom.pose.pose.position.y = y_dot;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_dot);

    odom.twist.twist.linear.x = V_x;
    odom.twist.twist.linear.y = V_y;
    odom.twist.twist.angular.z = omega;

    RobotOdometry::publishAsOdom(odom);
}
