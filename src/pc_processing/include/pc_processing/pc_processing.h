#include <ros/ros.h>
#include <ros/console.h>
#include <ctime>
#include <string> 
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
#include <pcl/filters/extract_indices.h>
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

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

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
			double plane_threshold_dist;
			double plane_filtering;
			int plane_max_it;
			// double plane_axis_x;
			// double plane_axis_y;
			// double plane_axis_z;
			// double plane_angle_th;

	// tf listener
		const tf::TransformListener* tf_listener;

	// point clouds
		sensor_msgs::PointCloud2* full_pc;
		sensor_msgs::PointCloud2* filtered_pc;
		sensor_msgs::PointCloud2* segmented_pc;

	// planes
		tf::Vector3 plane_center;
		tf::Quaternion plane_quat;
		sensor_msgs::PointCloud2* plane_pc;

public:
			bool plane_detection_enable;
	// constr/destr
		PcProcessing();
		virtual ~PcProcessing();

	// setters
		void set_subsize(double subsize);
		void set_filtering_params(bool filtering, double filter_radius, int filter_min_neighbors);
		void set_cutting_params(bool cutting_x_enable, float cutting_x_min, float cutting_x_max, bool cutting_y_enable, float cutting_y_min, float cutting_y_max, bool cutting_z_enable, float cutting_z_min, float cutting_z_max);
		void set_plane_detection_params(bool plane_detection, double dist_th, double filtering, int max_it); //, double axis_x, double plane_axis_y, double plane_axis_z, double angle_th);
		void set_listener(const tf::TransformListener* listener);

	// getters
		sensor_msgs::PointCloud2* get_filtered_pc();
		sensor_msgs::PointCloud2* get_full_pc();
		sensor_msgs::PointCloud2* get_plane_pc();

	// pc_processing
		void merge_pc(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2);
		void subsample_pc();
		void filter_pc();
		void cutting_pc();
	
	// plane detection
		void initialize_seg_pc();
		void plane_detection(int plane_n);
};