#include <differential.h>
#include <ackerman.h>

#include <dynamic_reconfigure/server.h>
#include <ros_project_a/config_paramsConfig.h>

void reconfigureOdometryParams(const ros_project_a::config_paramsConfig &params, uint32_t level, RobotOdometry &d_odom, RobotOdometry &a_odom)
{
    d_odom.setPositionX(params.pos_x_param);
    d_odom.setPostionY(params.pos_y_param);
    a_odom.setPositionX(params.pos_x_param);
    a_odom.setPostionY(params.pos_y_param);

    if (params.odometry_type == OdometryType::DIFFERENTIAL) {
        ROS_INFO("Reconfigure parameters for Differential drive => X: %f Y: %f", params.pos_x_param, params.pos_y_param);
        d_odom.activate(true);
        a_odom.activate(false);
    } else if(params.odometry_type == OdometryType::ACKERMAN) {
        ROS_INFO("Reconfigure parameters for Ackerman => X: %f Y: %f", params.pos_x_param, params.pos_y_param);
        a_odom.activate(true);
        d_odom.activate(false);
    }else{
        ROS_INFO("Reconfigure parameters for BOTH systems => X: %f Y: %f", params.pos_x_param, params.pos_y_param);
        d_odom.activate(true);
        a_odom.activate(true);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform");

    dynamic_reconfigure::Server<ros_project_a::config_paramsConfig> server;

    Differential dd_odometry(0, 0, 0);
    Ackerman ak_odometry(0, 0, 0);

    server.setCallback(boost::bind(&reconfigureOdometryParams, _1, _2, std::ref(dd_odometry), std::ref(ak_odometry)));

    ros::spin();
}




