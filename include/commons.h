#ifndef PROJECT_COMMONS_H
#define PROJECT_COMMONS_H

/********************************
 * INCLUDES
 */

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include "ros_project_a/floatStamped.h"

/**
 * CONSTANTS
 */

/********************************
 * TYPE DEFINITIONS
 */

typedef ros_project_a::floatStamped FloatStamped;
typedef ros_project_a::floatStampedConstPtr FloatStampedConstPtr;
typedef ros_project_a::floatStampedPtr FloatStampedPtr;

typedef message_filters::sync_policies::ApproximateTime<FloatStamped, FloatStamped, FloatStamped> SyncPolicy;


#define DEBUG

#endif //PROJECT_COMMONS_H
