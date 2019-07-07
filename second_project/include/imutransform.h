#ifndef SECOND_PROJECT_IMUTRANSFORM_H
#define SECOND_PROJECT_IMUTRANSFORM_H


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuTransform {
private:
    /**
     * ROS
     */
    ros::NodeHandle nh;
    ros::Publisher p_imu;
    ros::Subscriber s_imu;

    /**
     * Constants
     */
    const std::string FRAME_ID = "base_link";

    /**
     * Private members
     */
    void transform(const sensor_msgs::ImuConstPtr &imu);

public:
    ImuTransform();

};


#endif //SECOND_PROJECT_IMUTRANSFORM_H
