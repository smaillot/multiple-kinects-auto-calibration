#include <string>
#include <vector>

#include "ros/ros.h"
#include <ros/console.h>
#include "Cloud.h"
#include "tf_node.h"
#include "Preprocessing.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <calib/MergingConfig.h>

void cut(pc_t& input, float ymin, float ymax);
