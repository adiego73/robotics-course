#include <odometry_differential.h>

#include <dynamic_reconfigure/server.h>
#include <ros_project_a/config_paramsConfig.h>

void reconfigureParams(ros_project_a::config_paramsConfig& params, uint32_t level, OdometryDifferential* odom){
    odom->setPositionX(params.pos_x_param);
    odom->setPostionY(params.pos_y_param);

    odom->activate(params.odometry_type == OdometryType::DIFFERENTIAL || params.odometry_type == OdometryType::BOTH);

    ROS_INFO("Reconfigure parameters %d", params.odometry_type);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transform");

    dynamic_reconfigure::Server<ros_project_a::config_paramsConfig> server;

    OdometryDifferential odometryDifferential(0,0,0);

    server.setCallback(boost::bind(&reconfigureParams, _1, _2, &odometryDifferential));

    ros::spin();
}




