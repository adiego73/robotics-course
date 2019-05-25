//
// Created by diego on 5/17/19.
//

#ifndef PROJECT_CODOMETRY_H
#define PROJECT_CODOMETRY_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include "commons.h"

class RobotOdometry {
protected:
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
    ros::Time time_ = ros::Time(0,0);

    bool active = true;
    /**
     * ROS
     */
    ros::NodeHandle n;
    ros::Publisher p_odom;

    message_filters::Subscriber<FloatStamped> speed_r;
    message_filters::Subscriber<FloatStamped> speed_l;
    message_filters::Subscriber<FloatStamped> steer;

    message_filters::Synchronizer<SyncPolicy> sync;

    tf::TransformBroadcaster broadcaster;

    /**
     * Constants
     */
    const int STEERING_FACTOR = 18;

    /**
     * Virtual members
     */
    virtual void broadcastTransform();

    void publishAsOdom(std::string base_link_name);

    double deg2rad(double degrees);

    RobotOdometry(double pos_x, double pos_y, double theta);

public:
    virtual void calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer) = 0;

    void setPositionX(double x);

    void setPostionY(double y);

    void activate(bool flag);

};


#endif //PROJECT_CODOMETRY_H
