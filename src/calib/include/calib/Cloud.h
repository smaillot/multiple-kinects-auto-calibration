#ifndef DEF_CLOUD
#define DEF_CLOUD

#include <string>
#include <ros/console.h>

#include <shape_msgs/Plane.h>
#include <calib/Planes.h>
#include <calib/PlaneClouds.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "calib/pc_conv.h"

#include <dynamic_reconfigure/server.h>
#include <calib/CloudConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;

struct param_voxel_t
{
	bool enable;
	float x;
	float y;
	float z;
};
struct cut_axis_t
{
	bool enable;
	float bounds[2];
};
struct param_cut_t
{
	cut_axis_t x;
	cut_axis_t y;
	cut_axis_t z;
};
struct param_transform_t
{
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
};
struct param_plane_t
{
	int method;
	int n_planes;
	double th_dist;
	int max_it;
};

class Cloud
{
private:
	ros::NodeHandle* node;
	tf::TransformListener *tf_listener;

	ros::Subscriber sub;
	ros::Publisher pub_raw;
	ros::Publisher pub_preproc;
	ros::Publisher pub_planes_pc_col;
	ros::Publisher pub_planes_pc;
	ros::Publisher pub_planes;

	pcl::VoxelGrid<pcl::PCLPointCloud2> filter_voxel;
    pcl::PassThrough<pcl::PointXYZRGB> filter_cut;
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	param_voxel_t param_voxel;
	param_cut_t param_cut;
	param_transform_t param_transform;
	param_plane_t param_plane;

	vector <Eigen::Vector4f> planes;
	

public:
	Cloud(ros::NodeHandle* node, string sub_name, string pub_name);

	void reset_params();

	void publish(ros::Publisher& pub, sensor_msgs::PointCloud2& msg, string frame);
	void publish(ros::Publisher& pub, const sensor_msgs::PointCloud2ConstPtr& msg_ptr, string frame);
	void publish(ros::Publisher& pub, const pcl::PCLPointCloud2Ptr& msg_ptr, string frame);

	void conf_callback(calib::CloudConfig &config, uint32_t level);
	void update(const sensor_msgs::PointCloud2ConstPtr& input);

	Eigen::Matrix4f get_transform(param_transform_t params, bool rot, bool tr);
	pcl::PCLPointCloud2Ptr subsample(const pcl::PCLPointCloud2Ptr& input, param_voxel_t params);
	pcl::PCLPointCloud2Ptr cut(pcl::PCLPointCloud2Ptr input, param_cut_t params);
	vector <Eigen::Vector4f> detect_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, param_plane_t params);
	void colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float ratio);
};

#endif