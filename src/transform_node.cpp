#include <odometry_differential.h>
#include <odometry_ackerman.h>

#include <dynamic_reconfigure/server.h>
#include <ros_project_a/config_paramsConfig.h>

void reconfigureOdometryParams(ros_project_a::config_paramsConfig &params, uint32_t level, OdometryDifferential *d_odom,
                               OdometryAckerman *a_odom){

    if(params.odometry_type == OdometryType::DIFFERENTIAL) {
        ROS_INFO("Reconfigure parameters for Differential drive => X: %f Y: %f", params.pos_x_param, params.pos_y_param);
        d_odom->setPositionX(params.pos_x_param);
        d_odom->setPostionY(params.pos_y_param);
        d_odom->activate(true);
        a_odom->activate(false);
    }
    else {
        ROS_INFO("Reconfigure parameters for Ackerman => X: %f Y: %f", params.pos_x_param, params.pos_y_param);
        a_odom->setPositionX(params.pos_x_param);
        a_odom->setPostionY(params.pos_y_param);
        a_odom->activate(true);
        d_odom->activate(false);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "transform");

    dynamic_reconfigure::Server<ros_project_a::config_paramsConfig> server;

    OdometryDifferential odometryDifferential(0,0,0);
    OdometryAckerman odometryAckerman(0,0,0);

    server.setCallback(boost::bind(&reconfigureOdometryParams, _1, _2, &odometryDifferential, &odometryAckerman));

    ros::spin();
}




