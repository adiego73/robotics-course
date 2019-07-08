#ifndef SECOND_PROJECT_GPSTRANSFORM_H
#define SECOND_PROJECT_GPSTRANSFORM_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

class GPSTransform {
private:
    /**
     * ROS
     */
    ros::NodeHandle nh;
    ros::Publisher p_gps;
    ros::Subscriber s_gps;

    /**
     * Constants
     */
    const std::string FRAME_ID = "base_link";

    /**
     * Private members
     */
    void transform(const sensor_msgs::NavSatFixConstPtr &gps);

public:
    GPSTransform();
};


#endif //SECOND_PROJECT_GPSTRANSFORM_H
