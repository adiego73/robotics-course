#include "imutransform.h"

ImuTransform::ImuTransform()
{
    s_imu = nh.subscribe("/swiftnav/rear/imu", 100, &ImuTransform::transform, this);
    p_imu = nh.advertise<sensor_msgs::Imu>("/car/imu", 50);
}

void ImuTransform::transform(const sensor_msgs::ImuConstPtr &imu)
{
    sensor_msgs::Imu data;

    data.angular_velocity = imu->angular_velocity;
    data.angular_velocity_covariance = std::move(imu->angular_velocity_covariance);
    data.linear_acceleration = std::move(imu->linear_acceleration);
    data.linear_acceleration_covariance = std::move(imu->linear_acceleration_covariance);
    data.orientation = std::move(imu->orientation);
    data.orientation_covariance = std::move(data.orientation_covariance);

    data.header.frame_id = FRAME_ID;
    data.header.stamp = ros::Time::now();

    p_imu.publish(data);
}