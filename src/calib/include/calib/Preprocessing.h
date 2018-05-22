#ifndef DEF_PREPROCESSING
#define DEF_PREPROCESSING

#include "Cloud.h"
#include <dynamic_reconfigure/server.h>
#include <calib/PreprocessingConfig.h>

using namespace std;

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

class Preprocessing : public Cloud
{
private:

	ros::NodeHandle* node;
	tf::TransformListener *tf_listener;
	string frame;
	string frame_pub;

	ros::Subscriber sub;
	ros::Publisher pub_preproc;

	void publish(ros::Publisher& pub, pc_t& msg);
	void publish(ros::Publisher& pub, const pcConstPtr& msg_ptr);
	void publish(ros::Publisher& pub, const pc_msg_t& msg);
	
	pcl::VoxelGrid<Point> filter_voxel;
    pcl::PassThrough<Point> filter_cut;

	param_voxel_t param_voxel;
	param_cut_t param_cut;

public:
	Preprocessing(ros::NodeHandle* node, Cloud* cloud);
	void update(const pcConstPtr& input);
	void conf_callback(calib::PreprocessingConfig &config, uint32_t level);

	pcPtr subsample(const pcPtr& input, param_voxel_t params);
	pcPtr cut(const pcPtr input, param_cut_t params);
	
};

#endif