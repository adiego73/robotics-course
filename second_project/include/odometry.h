#ifndef ACKERMAN_ODOM_H
#define ACKERMAN_ODOM_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

class Odometry {
private:
    /**
 * Odometry parameters
 */
    double x_dot = 0;
    double y_dot = 0;
    double theta_dot = 0;

    double V = 0;
    double V_x = 0;
    double V_y = 0;
    double omega = 0;

//    long int time_ = 0;
    ros::Time time_ = ros::Time(0, 0);

    /**
     * ROS
     */
    ros::NodeHandle n;
    ros::Publisher p_odom;

    ros::Subscriber speedsteer;
    tf::TransformBroadcaster broadcaster;

    /**
     * Constants
     */
    const std::string CHILD_FRAME_ID = "base_link";
    const std::string FRAME_ID = "odom";
    const int STEERING_FACTOR = 18;
    const double FRONT_REAR_DISTANCE = 1.765; // from cm (176.5) to m

    /**
     * Private members
     */
    void calculate(const geometry_msgs::PointStampedConstPtr &speed_steer);

public:
    Odometry(double pos_x, double pos_y, double theta);

    void broadcastTransform();

    void publishAsOdom();

    double deg2rad(double degrees);

    double kmph2mps(double speed_km_per_hour);
};


#endif // ACKERMAN_ODOM_H