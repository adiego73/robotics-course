#include <odometry_differential.h>

OdometryDifferential::OdometryDifferential(double pos_x, double pos_y, double theta) : pos_x(pos_x), pos_y(pos_y) {
}

void OdometryDifferential::publishAsTf() {
    tf::Quaternion q;
    q.setRPY(0, 0, this->theta);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(this->pos_x, this->pos_y, 0));
    transform.setRotation(q);

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/transform", "/differential_odometry"));
}

void OdometryDifferential::publishAsOdom() {

}

void OdometryDifferential::calculateDifferentialDrive(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer) {

    long int timeDiff = V_r->header.stamp.sec - time_;
    time_ = V_r->header.stamp.sec;

    double angle = steer->data / STEERING_FACTOR;

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
        y = pos_y - (V / omega) * (std::cos(angle) - std::cos(theta));
        x = pos_x + (V / omega) * (std::sin(angle) - std::sin(theta));
    }

    theta = theta + (omega * timeDiff);
    pos_x = x;
    pos_y = y;

#ifdef DEBUG
    std::string posX = std::to_string(pos_x) + " X ";
    std::string posY = std::to_string(pos_y) + " Y ";
    std::string thetaS = std::to_string(theta) + " T ";
    ROS_INFO( posX.c_str());
    ROS_INFO(posY.c_str());
    ROS_INFO(thetaS.c_str());
#endif

    this->publishAsTf();
    this->publishAsOdom();
}


