#ifndef PROJECT_ODOMETRY_H
#define PROJECT_ODOMETRY_H

#include <commons.h>
#include "robot_odometry.h"

class Differential : public RobotOdometry {
private:
    const int REAR_WHEELS_BASE_LINE = 130;

public:
    Differential(double pos_x, double pos_y, double theta);

    void calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer) override;
};


#endif //PROJECT_ODOMETRY_H
