#ifndef PROJECT_COMMONS_H
#define PROJECT_COMMONS_H

/********************************
 * INCLUDES
 */

#include <ros/ros.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "first_project/floatStamped.h"
#include "first_project/robotOdometry.h"

/**
 * CONSTANTS
 */

/********************************
 * TYPE DEFINITIONS
 */

typedef first_project::floatStamped FloatStamped;
typedef first_project::floatStampedConstPtr FloatStampedConstPtr;

typedef message_filters::sync_policies::ApproximateTime<FloatStamped, FloatStamped, FloatStamped> SyncPolicy;

enum OdometryType {
    DIFFERENTIAL = 0, ACKERMAN = 1
};


//#define DEBUG

#endif //PROJECT_COMMONS_H
