// ros
	#include <ros/console.h>
	#include "ros/ros.h"
// std
	#include <string>
	#include <vector>
// point cloud
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_conversions/pcl_conversions.h>
// my libraries
	#include <geometry/PointCloud.h>
// dynamic reconfigure
	#include <dynamic_reconfigure/server.h>
	#include <registration/SubSamplingConfig.h>
	#include <registration/CuttingConfig.h>
	#include <registration/RadiusFilteringConfig.h>
	#include <registration/OutliersRemovalConfig.h>
	

// config callback
	void subsampling_conf_callback(registration::SubSamplingConfig &config, uint32_t level);
	void cutting_conf_callback(registration::CuttingConfig &config, uint32_t level);
	void radius_filtering_conf_callback(registration::RadiusFilteringConfig &config, uint32_t level);