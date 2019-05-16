#include <odometry_differential.h>

//void calculateDifferentialDrive(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer, tf::TransformBroadcaster& br) {
//
//    long int timeDiff = V_r->header.stamp.sec - time_;
//    time_ = V_r->header.stamp.sec;
//
//    double angle = steer->data / STEERING_FACTOR;
//
//    V = (V_r->data + V_l->data) / 2.0;
//    omega = (V_r->data - V_l->data) / REAR_WHEELS_BASE_LINE;
//
//    double x = 0;
//    double y = 0;
//
//    // integration of pos_y
//    if (omega <= 0.2) {
//        // 2nd Runge-Kutta
//        // TODO: add time diff => y_k+1 = y_k + v_k * Ts * sin(theta_k + (omega_k * Ts / 2))
//        y = pos_y + V * timeDiff * (std::sin(theta + (omega * timeDiff / 2)));
//        x = pos_x + V * timeDiff * (std::cos(theta + (omega * timeDiff / 2)));
//    } else {
//        // exact integration of y_k+1
//        y = pos_y - (V / omega) * (std::cos(angle) - std::cos(theta));
//        // exact integration of x_k+1
//        x = pos_x + (V / omega) * (std::sin(angle) - std::sin(theta));
//    }
//
//    theta = theta + (omega * timeDiff);
//    pos_x = x;
//    pos_y = y;
//
//    std::string posX = std::to_string(pos_x) + " X ";
//    std::string posY = std::to_string(pos_y) + " Y ";
//    std::string thetaS = std::to_string(theta) + " T ";
//    ROS_INFO( posX.c_str());
//    ROS_INFO(posY.c_str());
//    ROS_INFO(thetaS.c_str());
//
//    tf::Quaternion q;
//    q.setRPY(0, 0, theta);
//
//    tf::Transform transform;
//    transform.setOrigin(tf::Vector3(pos_x, pos_y, 0));
//    transform.setRotation(q);
//
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/transform", "/differential_odometry"));
//}

//void calculateAckerman(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer, tf::TransformBroadcaster& br) {
//
//    long int timeDiff = V_r->header.stamp.sec - time_;
//    time_ = V_r->header.stamp.sec;
//
//    double angle = steer->data / STEERING_FACTOR;
//
////    R = FRONT_REAR_DISTANCE / std::tan(steer->data);
//    V = (V_r->data + V_l->data) / 2.0;
//
//    omega = V * std::tan(angle) / FRONT_REAR_DISTANCE;
//
////    Vf = (omega * REAR_WHEELS_BASE_LINE) / std::sin(angle);
//
//    double x = 0;
//    double y = 0;
//
//    x = pos_x + V * std::cos(theta) * timeDiff;
//    y = pos_y + V * std::sin(theta) * timeDiff;
//
//    theta = theta + omega * timeDiff;
//
//    pos_x = x;
//    pos_y = y;
//
//    std::string posX = std::to_string(pos_x) + " X ";
//    std::string posY = std::to_string(pos_y) + " Y ";
//    std::string thetaS = std::to_string(theta) + " T ";
//    ROS_INFO( posX.c_str());
//    ROS_INFO(posY.c_str());
//    ROS_INFO(thetaS.c_str());
//
////    tf::Quaternion q;
////    q.setRPY(0, 0, theta);
////
////    tf::Transform transform;
////    transform.setOrigin(tf::Vector3(pos_x, pos_y, 0));
////    transform.setRotation(q);
////
////    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/transform", "/ackerman_odometry"));
//}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transform");

    ros::NodeHandle n;

    OdometryDifferential odometryDifferential(0,0,0);

    message_filters::Subscriber<FloatStamped> speed_r(n, "/speedR_stamped", 1);
    message_filters::Subscriber<FloatStamped> speed_l(n, "/speedL_stamped", 1);
    message_filters::Subscriber<FloatStamped> steer(n, "/steer_stamped", 1);

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), speed_r, speed_l, steer);


    sync.registerCallback( boost::bind( &OdometryDifferential::calculateDifferentialDrive, &odometryDifferential, _1, _2, _3) ); // OdometryDifferential::calculateDifferentialDrive, this, _1, _2, _3

    ros::spin();
}




