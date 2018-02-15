#include <ros/ros.h>
#include <ros/console.h>
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
#include <pcl/filters/radius_outlier_removal.h>
// synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// messages
#include <sensor_msgs/PointCloud2.h>
// tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <pc_processing/registrationConfig.h>


class PcProcessing
{
private:
	// parameters
	double subsize;
	
	bool filtering;
	double filter_radius;
	int filter_min_neighbors;

	// tf listener
	const tf::TransformListener* tf_listener;

	// point clouds
	sensor_msgs::PointCloud2* full_pc;
	sensor_msgs::PointCloud2* filtered_pc;

public:
	PcProcessing();
	virtual ~PcProcessing();
	void merge_pc(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2);
	void subsample_pc();
	void set_subsize(double subsize);
	void set_filtering_params(bool filtering, double filter_radius, int filter_min_neighbors);
	double get_subsize();
	void set_listener(const tf::TransformListener* listener);
	sensor_msgs::PointCloud2* get_filtered_pc();
	void filter_pc();
};