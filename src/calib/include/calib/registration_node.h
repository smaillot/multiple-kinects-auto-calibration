#include <string>
#include "calib/Cloud.h"
#include "calib/Merging.h"
#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
#include <calib/RegistrationConfig.h>
#include <calib/MergingConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>