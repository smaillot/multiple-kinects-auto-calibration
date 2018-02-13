#include <ros/ros.h>
#include <ctime>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
// synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <multiple_kinects/subsamplingConfig.h>
// My libraries
#include <pc_processing/pc_processing.h>