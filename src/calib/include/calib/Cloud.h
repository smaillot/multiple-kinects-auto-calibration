#ifndef DEF_CLOUD
#define DEF_CLOUD

#include <string>
#include <ros/console.h>

#include <shape_msgs/Plane.h>
#include <calib/Planes.h>
#include <calib/PlaneClouds.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <calib/CloudConfig.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> pc_t;
typedef pc_t::Ptr pcPtr;
typedef pc_t::ConstPtr pcConstPtr;
typedef sensor_msgs::PointCloud2 pc_msg_t;
typedef pc_msg_t::Ptr pc_msgPtr;
typedef pcl::PointCloud<pcl::Normal> pc_n_t;
typedef pc_n_t::Ptr pc_nPtr;
typedef pcl::VFHSignature308 global_desc_t;
typedef pcl::PointCloud<global_desc_t> plane_feat_t;
typedef plane_feat_t::Ptr pc_featPtr;
/**/typedef pcl::FPFHSignature33 kp_t;/**/
/*/ typedef pcl::VFHSignature308 kp_t; /**/
/*/ typedef pcl::SHOT352 kp_t; /**/
typedef pcl::PointCloud<kp_t> kp_feat_t;
typedef kp_feat_t::Ptr kp_featPtr;

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

void MSGtoPCL(const pc_msg_t& msg, pcPtr& cloud);
void colorize(pcPtr input, float ratio);

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

	pcl::VoxelGrid<Point> filter_voxel;
    pcl::PassThrough<Point> filter_cut;
	pcl::SACSegmentation<Point> seg;
    pcl::ExtractIndices<Point> extract;

	param_voxel_t param_voxel;
	param_cut_t param_cut;
	param_transform_t param_transform;
	param_plane_t param_plane;

	vector <Eigen::Vector4f> planes;
	

public:
	Cloud(ros::NodeHandle* node, string sub_name, string pub_name);

	void reset_params();

	void publish(ros::Publisher& pub, pc_t& msg, string frame);
	void publish(ros::Publisher& pub, const pcConstPtr& msg_ptr, string frame);
	void publish(ros::Publisher& pub, const pc_msg_t& msg, string frame);

	void conf_callback(calib::CloudConfig &config, uint32_t level);
	void update(const pcConstPtr& input);

	tf::Transform get_transform(param_transform_t params, bool rot, bool tr);
	pcPtr subsample(const pcPtr& input, param_voxel_t params);
	pcPtr cut(const pcPtr input, param_cut_t params);
	vector <Eigen::Vector4f> detect_plane(const pcPtr& input, param_plane_t params);
};

#endif