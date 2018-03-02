#ifndef GEOMETRY_POINTCLOUD
#define GEOMETRY_POINTCLOUD

// std
	#include <string>
	#include <iostream>
// ros
	#include <ros/console.h>
// point cloud
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/impl/point_types.hpp>
	#include <pcl_ros/transforms.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/passthrough.h>
	#include <pcl/filters/radius_outlier_removal.h>
	#include <sensor_msgs/PointCloud2.h>
// tf
	#include <tf/LinearMath/Transform.h>
	#include <tf_conversions/tf_eigen.h>
	#include <tf/transform_listener.h>
	#include <tf/transform_broadcaster.h>

const std::string REFERENCE_FRAME = "cam_center";

struct cutting_axis_t
{
	bool enable;
	float bounds[2];
};
struct cutting_params_t
{
	cutting_axis_t x;
	cutting_axis_t y;
	cutting_axis_t z;
};
struct subsampling_params_t
{
	bool enable;
	float x;
	float y;
	float z;
};
struct radius_filtering_params_t
{
	bool enable;
	float radius;
	int min_neighbors;
};
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

namespace geometry
{
	class PointCloud
	{
		private:
			
			// ros node interaction
				ros::NodeHandle node;  
				const tf::TransformListener* tf_listener;

				std::string sub_name;  
				std::string pub_name;
				ros::Subscriber pc_sub;
				ros::Publisher pc_pub;
			
			// point cloud
				pcl::PCLPointCloud2* cloud;
				pcl::VoxelGrid<pcl::PCLPointCloud2> filter_voxel;
    			pcl::PassThrough<pcl::PointXYZRGB> filter_cut;
				pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> filter_radius;

			// parameters
				subsampling_params_t subsampling_params;
				cutting_params_t cutting_params;
				radius_filtering_params_t radius_filtering_params;

		public:

			// constructor
				PointCloud(ros::NodeHandle nh, std::string subscribe_name, std::string publish_name);
					
			// getters
				sensor_msgs::PointCloud2* get_pc();

			// setters
				void set_subsampling_params(subsampling_params_t subsamples_params);
				void set_cutting_params(cutting_params_t cutting_params);
				void set_radius_filtering_params(radius_filtering_params_t radius_filtering_params);

			// update
				void update(const sensor_msgs::PointCloud2ConstPtr& cloud);

			// processing
				void subsample();
				void radius_filter();
				void cut();
	};
}

#endif