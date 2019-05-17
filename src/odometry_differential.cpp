#include <odometry_differential.h>

OdometryDifferential::OdometryDifferential(double pos_x, double pos_y, double theta) : pos_x(pos_x), pos_y(pos_y),
                                                                                       theta(theta),
                                                                                       speed_r(n, "/speedR_stamped", 1),
                                                                                       speed_l(n, "/speedL_stamped", 1),
                                                                                       steer(n, "/steer_stamped", 1),
                                                                                       sync(SyncPolicy(10), speed_r,
                                                                                            speed_l, steer) {

    sync.registerCallback(boost::bind(&OdometryDifferential::calculateDifferentialDrive, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom/differential", 50);
}

void OdometryDifferential::broadcastTransform() {
    broadcaster.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "transform", "differential"));
}

void OdometryDifferential::calculateDifferentialDrive(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l,
                                                      const FloatStampedConstPtr &steer) {

//    ros::Time current_time = ros::Time::now();
    long int dt = (V_r->header.stamp.sec - time_);
    time_ = V_r->header.stamp.sec;

    double alpha = steer->data / STEERING_FACTOR;

    V = (V_r->data + V_l->data) / 2.0;
    omega = (V_r->data - V_l->data) / REAR_WHEELS_BASE_LINE;

    double x = 0;
    double y = 0;

    // integration of pos_y
    if (omega <= 0.2) {
        // 2nd Runge-Kutta
        y = pos_y + V * dt * (std::sin(theta + (omega * dt / 2)));
        x = pos_x + V * dt * (std::cos(theta + (omega * dt / 2)));
    } else {
        // exact integration of x and y
        y = pos_y - (V / omega) * (std::cos(alpha) - std::cos(theta));
        x = pos_x + (V / omega) * (std::sin(alpha) - std::sin(theta));
    }

    ROS_INFO("%f", pos_y);

    theta = theta + (omega * dt);
    pos_x = x;
    pos_y = y;

    V_x = V * std::cos(alpha);
    V_y = V * std::sin(alpha);

    tf::Quaternion q;
    q.setRPY(0, 0, this->theta);

    this->transform.setOrigin(tf::Vector3(this->pos_x, this->pos_y, 0));
    this->transform.setRotation(q);

#ifdef DEBUG
    ROS_INFO("Position in X: %f", pos_x);
    ROS_INFO("Position in Y: %f", pos_y);
    ROS_INFO("Steering angle: %f", theta);
#endif

    if(this->active) {
        this->broadcastTransform();
        this->publishAsOdom();
    }

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

void OdometryDifferential::setPositionX(double x) {
    this->pos_x = x;
}

void OdometryDifferential::setPostionY(double y) {
    this->pos_y = y;
}

void OdometryDifferential::activate(bool flag) {
    this->active = flag;
}

