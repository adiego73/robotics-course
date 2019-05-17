#include <odometry_differential.h>

OdometryDifferential::OdometryDifferential(double pos_x, double pos_y, double theta) : pos_x(pos_x), pos_y(pos_y),
                                                                                       theta(theta),
                                                                                       speed_r(n, "/speedR_stamped", 1),
                                                                                       speed_l(n, "/speedL_stamped", 1),
                                                                                       steer(n, "/steer_stamped", 1),
                                                                                       sync(SyncPolicy(10), speed_r,
                                                                                            speed_l, steer) {

    sync.registerCallback(boost::bind(&OdometryDifferential::calculateDifferentialDrive, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom", 50);
}

void OdometryDifferential::broadcastTransform() {
    broadcaster.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "/transform", "/differential_odometry"));
}

void OdometryDifferential::calculateDifferentialDrive(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l,
                                                      const FloatStampedConstPtr &steer) {

    long int timeDiff = V_r->header.stamp.sec - time_;
    time_ = V_r->header.stamp.sec;

    double alpha = steer->data / STEERING_FACTOR;

    V = (V_r->data + V_l->data) / 2.0;
    omega = (V_r->data - V_l->data) / REAR_WHEELS_BASE_LINE;

    double x = 0;
    double y = 0;

    // integration of pos_y
    if (omega <= 0.2) {
        // 2nd Runge-Kutta
        y = pos_y + V * timeDiff * (std::sin(theta + (omega * timeDiff / 2)));
        x = pos_x + V * timeDiff * (std::cos(theta + (omega * timeDiff / 2)));
    } else {
        // exact integration of x and y
        y = pos_y - (V / omega) * (std::cos(alpha) - std::cos(theta));
        x = pos_x + (V / omega) * (std::sin(alpha) - std::sin(theta));
    }

    theta = theta + (omega * timeDiff);
    pos_x = x;
    pos_y = y;

    V_x = V * std::cos(alpha);
    V_y = V * std::sin(alpha);

    tf::Quaternion q;
    q.setRPY(0, 0, this->theta);

    this->transform.setOrigin(tf::Vector3(this->pos_x, this->pos_y, 0));
    this->transform.setRotation(q);

#ifdef DEBUG
    std::string posX = std::to_string(pos_x) + " X ";
    std::string posY = std::to_string(pos_y) + " Y ";
    std::string thetaS = std::to_string(theta) + " T ";
    ROS_INFO(posX.c_str());
    ROS_INFO(posY.c_str());
    ROS_INFO(thetaS.c_str());
#endif

    this->broadcastTransform();
    this->publishAsOdom();

}

void OdometryDifferential::publishAsOdom() {

    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "dd_odom";

    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    odom.twist.twist.linear.x = V_x;
    odom.twist.twist.linear.y = V_y;
    odom.twist.twist.angular.z = omega;

    p_odom.publish(odom);
}


