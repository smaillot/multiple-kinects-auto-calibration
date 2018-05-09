#ifndef GEOMETRY_POINTCLOUD
#define GEOMETRY_POINTCLOUD

// std
	#include <string>
	#include <iostream>
	#include <math.h>
    #include <Eigen/Core>
// ros
	#include <ros/console.h>
	#include <ros/callback_queue.h>
	#include <ros/callback_queue_interface.h>
// point cloud
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/impl/point_types.hpp>
	#include <pcl_ros/transforms.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/passthrough.h>
	#include <pcl/filters/radius_outlier_removal.h>
	#include <pcl/filters/statistical_outlier_removal.h>
	#include <pcl/keypoints/iss_3d.h>
	#include <pcl/keypoints/sift_keypoint.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl/kdtree/kdtree_flann.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/registration/correspondence_estimation.h>
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
struct outliers_removal_params_t
{
	bool enable;
	int meank;
	float std_mul;
};
struct kp_params_t
{
	int method;
	float support_radius;
	float nms_radius;
	float min_scale;
	int nr_octave;
	int nr_scales_per_oct;
	float min_contrast;
	float nrad;
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
				std::string frame;

				std::string sub_name;  
				std::string pub_name;
				ros::Subscriber pc_sub;
				ros::Publisher pc_pub;
				ros::Publisher pc_pub_raw;
				ros::Publisher kp_pub;
			
			// point cloud
				pcl::VoxelGrid<pcl::PCLPointCloud2> filter_voxel;
    			pcl::PassThrough<pcl::PointXYZRGB> filter_cut;
				pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> filter_radius;
				pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
				pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss;
				pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
				pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
				pcl::search::KdTree<pcl::PointNormal> tree_n;
				pcl::search::KdTree<pcl::PointXYZRGB> tree;


			// parameters
				subsampling_params_t subsampling_params;
				cutting_params_t cutting_params;
				radius_filtering_params_t radius_filtering_params;
				outliers_removal_params_t outliers_removal_params;
				kp_params_t kp_params;

		public:

			// constructor
				PointCloud(ros::NodeHandle nh, std::string subscribe_name, std::string publish_name);

			// setters
				void set_subsampling_params(subsampling_params_t subsamples_params);
				void set_cutting_params(cutting_params_t cutting_params);
				void set_radius_filtering_params(radius_filtering_params_t radius_filtering_params);
				void set_outliers_removal_params(outliers_removal_params_t outliers_removal_params);
				void set_kp_params(kp_params_t kp_params);
				void change_frame(std::string frame);

			// update
				void update(const sensor_msgs::PointCloud2ConstPtr& cloud);

			// processing
				pcl::PCLPointCloud2ConstPtr subsample(pcl::PCLPointCloud2ConstPtr cloudPtr);
				pcl::PCLPointCloud2ConstPtr cut(pcl::PCLPointCloud2ConstPtr cloudPtr, int kp);
				pcl::PCLPointCloud2ConstPtr radius_filter(pcl::PCLPointCloud2ConstPtr cloudPtr);
				pcl::PCLPointCloud2ConstPtr outliers_removal(pcl::PCLPointCloud2ConstPtr cloudPtr);
				pcl::PointCloud<pcl::PointNormal>::Ptr compute_normals(pcl::PCLPointCloud2ConstPtr cloudPtr);
				pcl::PCLPointCloud2ConstPtr keypoints(pcl::PCLPointCloud2ConstPtr cloudPtr);
	};
}

#endif