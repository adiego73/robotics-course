#ifndef PROJECT_ODOMETRY_A
#define PROJECT_ODOMETRY_A

#include <commons.h>
#include "robot_odometry.h"

class Ackerman : public RobotOdometry {
private:
    /**
     * Constants
     */
    const double FRONT_REAR_DISTANCE = 176.5;

public:
    Ackerman(double pos_x, double pos_y, double theta);

    void calculate(const FloatStampedConstPtr &V_r, const FloatStampedConstPtr &V_l, const FloatStampedConstPtr &steer) override;

    void publishAsOdom();
};


#endif //PROJECT_ODOMETRY_A
