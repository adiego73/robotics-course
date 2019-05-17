#include <ackerman.h>

Ackerman::Ackerman(double pos_x, double pos_y, double theta) : RobotOdometry(pos_x, pos_y, theta)
{

    sync.registerCallback(boost::bind(&Ackerman::calculate, this, _1, _2, _3));

    p_odom = n.advertise<nav_msgs::Odometry>("/car_odom/ackerman", 50);
}

void Ackerman::broadcastTransform()
{
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "transform", "ackerman"));
}

void Ackerman::calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer)
{

    double alpha = steer->data / STEERING_FACTOR;

    long int dt = V_r->header.stamp.sec - time_;
    time_ = V_r->header.stamp.sec;

    V = (V_r->data + V_l->data) / 2.0;
    omega = V * std::tan(alpha) / FRONT_REAR_DISTANCE;

    double x = 0;
    double y = 0;

    x = pos_x + V * std::cos(theta) * dt;
    y = pos_y + V * std::sin(theta) * dt;

    theta = theta + (omega * dt);
    pos_x = x;
    pos_y = y;

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

void Ackerman::publishAsOdom()
{

    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "ak_odom";

    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    odom.twist.twist.linear.x = V_x;
    odom.twist.twist.linear.y = V_y;
    odom.twist.twist.angular.z = omega;

    p_odom.publish(odom);
}
