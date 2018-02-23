// std
	#include <string>
	#include <iostream>
// ros
	#include <ros/console.h>
// point cloud
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/impl/point_types.hpp>
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_ros/transforms.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/radius_outlier_removal.h>
	#include <pcl/filters/passthrough.h>
// tf
	#include <tf/LinearMath/Transform.h>
	#include <tf_conversions/tf_eigen.h>
	#include <tf/transform_listener.h>
	#include <tf/transform_broadcaster.h>
// dynamic reconfigure
	#include <dynamic_reconfigure/server.h>
	#include <geometry/SubSamplingConfig.h>

const std::string REFERENCE_FRAME = "cam_center";

struct cutting_axis_t
{
	bool enable;
	float bounds[2];
};
struct cutting_param_t
{
	cutting_axis_t x;
	cutting_axis_t y;
	cutting_axis_t z;
};
struct subsampling_param_t
{
	bool enable;
	float x;
	float y;
	float z;
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
			ros::NodeHandle node;  
			std::string sub_name;  
			std::string pub_name;
			pcl::PCLPointCloud2* cloud;
        	const tf::TransformListener* tf_listener;
			pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
			dynamic_reconfigure::Server<geometry::SubSamplingConfig> subsampling_config_srv;  
			dynamic_reconfigure::Server<geometry::SubSamplingConfig>::CallbackType subsampling_config_cb;  
    		boost::recursive_mutex subsampling_config_mutex; 
			ros::Subscriber pc_sub;
			ros::Publisher pc_pub;

			// parameters
				subsampling_param_t subsampling_param;
				cutting_param_t cutting_param;
				// radius filtering
					bool filtering;
					float filter_radius;
					int filter_min_neighbors;
			// config callback
				void subsampling_conf_callback(geometry::SubSamplingConfig &config, uint32_t level);

		public:
			// constructors
				PointCloud(ros::NodeHandle nh, std::string subscribe_name, std::string publish_name);

			// getters
				sensor_msgs::PointCloud2* get_pc();


			// update
				void update(const sensor_msgs::PointCloud2ConstPtr& cloud);

			// processing
				void subsample();
				void radius_filter();
				void cut();
	};
}