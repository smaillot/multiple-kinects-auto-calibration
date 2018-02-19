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
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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
		// subsampling
			double subsize;
		//cutting
			bool cutting_x_enable;
			float cutting_x_min;
			float cutting_x_max;
			bool cutting_y_enable;
			float cutting_y_min;
			float cutting_y_max;
			bool cutting_z_enable;
			float cutting_z_min;
			float cutting_z_max;
		// filtering
			bool filtering;
			double filter_radius;
			int filter_min_neighbors;
		// plane detection
			bool plane_detection_enable;
			double plane_threshold_dist;
			double plane_filtering;

	// tf listener
		const tf::TransformListener* tf_listener;

	// point clouds
		sensor_msgs::PointCloud2* full_pc;
		sensor_msgs::PointCloud2* filtered_pc;

	// planes
		tf::Quaternion plane_quat;


public:
	// constr/destr
		PcProcessing();
		virtual ~PcProcessing();

	// setters
		void set_subsize(double subsize);
		void set_filtering_params(bool filtering, double filter_radius, int filter_min_neighbors);
		void set_cutting_params(bool cutting_x_enable, float cutting_x_min, float cutting_x_max, bool cutting_y_enable, float cutting_y_min, float cutting_y_max, bool cutting_z_enable, float cutting_z_min, float cutting_z_max);
		void set_plane_detection_params(bool plane_detection, double dist_th, double filtering);
		void set_listener(const tf::TransformListener* listener);

	// getters
		sensor_msgs::PointCloud2* get_filtered_pc();

	// pc_processing
		void merge_pc(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2);
		void subsample_pc();
		void filter_pc();
		void cutting_pc();
	
	// plane detection
		void plane_detection();
};