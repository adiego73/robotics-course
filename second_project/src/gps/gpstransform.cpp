#include "gpstransform.h"

GPSTransform::GPSTransform()
{
    s_gps = nh.subscribe("/swiftnav/rear/gps", 100, &GPSTransform::transform, this);
    p_gps = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 50);
}

void GPSTransform::transform(const sensor_msgs::NavSatFixConstPtr &gps)
{
    sensor_msgs::NavSatFix data;

    data.altitude = gps->altitude;
    data.latitude = gps->latitude;
    data.longitude = gps->longitude;
    data.position_covariance = std::move(gps->position_covariance);
    data.position_covariance_type = gps->position_covariance_type;
    data.status = std::move(gps->status);

    data.header.frame_id = FRAME_ID;
    data.header.stamp = ros::Time::now();

    p_gps.publish(data);
}
