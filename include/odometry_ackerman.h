#ifndef PROJECT_ODOMETRY_A
#define PROJECT_ODOMETRY_A

#include <commons.h>

class OdometryAckerman {
private:
    /**
     * Odometry parameters
     */
    double pos_x = 0;
    double pos_y = 0;
    double theta = 0;
    double V = 0;
    double V_x = 0;
    double V_y = 0;
    double omega = 0;
    long int time_ = 0;

    bool active;
    /**
     * ROS
     */
    ros::NodeHandle n;
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    ros::Publisher p_odom;

    message_filters::Subscriber<FloatStamped> speed_r;
    message_filters::Subscriber<FloatStamped> speed_l;
    message_filters::Subscriber<FloatStamped> steer;

    message_filters::Synchronizer<SyncPolicy> sync;
    /**
     * Constants
     */
    static const int REAR_WHEELS_BASE_LINE = 130;
    static const int STEERING_FACTOR = 18;
    static const int  FRONT_REAR_DISTANCE = 176.5;

    /**
     * Private functions
     */

    void broadcastTransform();
    void publishAsOdom();
//    void publishAsTf();

public:
    OdometryAckerman(double pos_x, double pos_y, double theta);
    void calculateAckermanDrive(const FloatStampedConstPtr& V_r, const FloatStampedConstPtr& V_l, const FloatStampedConstPtr& steer);
    void setPositionX(double x);
    void setPostionY(double y);
    void activate(bool flag);
};


#endif //PROJECT_ODOMETRY_A
