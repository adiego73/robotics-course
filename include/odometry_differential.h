//
// Created by diego on 5/16/19.
//

#ifndef PROJECT_ODOMETRY_H
#define PROJECT_ODOMETRY_H

#include <commons.h>
#include <tf/transform_broadcaster.h>

class OdometryDifferential {
private:
    /**
     * Odometry parameters
     */
    double pos_x = 0;
    double pos_y = 0;
    double theta = 0;
    double V = 0;
    double omega = 0;
    long int time_ = 0;

    /**
     * ROS
     */
    tf::TransformBroadcaster broadcaster;

    /**
     * Constants
     */
    static const int REAR_WHEELS_BASE_LINE = 130;
    static const int STEERING_FACTOR = 18;

    /**
     * Private functions
     */

    void publishAsTf();
    void publishAsOdom();

public:
    OdometryDifferential(double pos_x, double pos_y, double theta);
    void calculateDifferentialDrive(const FloatStampedConstPtr& V_r, const FloatStampedConstPtr& V_l, const FloatStampedConstPtr& steer);
};


#endif //PROJECT_ODOMETRY_H
