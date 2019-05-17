#include <odometry_differential.h>

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

    OdometryDifferential odometryDifferential(0,0,0);

    ros::spin();
}




