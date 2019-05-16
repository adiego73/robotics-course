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

#include "custom_messages/floatStamped.h"

/**
 * CONSTANTS
 */

/********************************
 * TYPE DEFINITIONS
 */

typedef custom_messages::floatStamped FloatStamped;
typedef custom_messages::floatStampedConstPtr FloatStampedConstPtr;
typedef custom_messages::floatStampedPtr FloatStampedPtr;

typedef message_filters::sync_policies::ApproximateTime<FloatStamped, FloatStamped, FloatStamped> SyncPolicy;


#define DEBUG

#endif //PROJECT_COMMONS_H
