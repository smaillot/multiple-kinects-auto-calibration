#ifndef DEF_CLOUD
#define DEF_CLOUD

#include <string>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <calib/CloudConfig.h>

#include <shape_msgs/Plane.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <calib/Planes.h>
#include <calib/PlaneClouds.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


using namespace std;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> pc_t;
typedef pc_t::Ptr pcPtr;
typedef pc_t::ConstPtr pcConstPtr;
typedef sensor_msgs::PointCloud2 pc_msg_t;
typedef pc_msg_t::Ptr pc_msgPtr;

struct param_transform_t
{
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
};

void MSGtoPCL(const pc_msg_t& msg, pcPtr& cloud);
void colorize(pcPtr input, float ratio);

class Cloud
{
private:
	param_transform_t param_transform;

protected:
	ros::NodeHandle* node;
	tf::TransformListener *tf_listener;
	string frame;
	string frame_pub;
	string name;

	ros::Subscriber sub;
	ros::Publisher pub_raw;

	pcPtr cloud;

public:
	Cloud();
	Cloud(ros::NodeHandle* node, string sub_name, string pub_name);

	void conf_callback(calib::CloudConfig &config, uint32_t level);
	void update(const pcConstPtr& input);

	void publish(ros::Publisher& pub, pc_t& msg);
	void publish(ros::Publisher& pub, const pcConstPtr& msg_ptr);
	void publish(ros::Publisher& pub, const pc_msg_t& msg);

	tf::Transform get_transform(param_transform_t params, bool rot, bool tr);
	pcPtr remove_nans(pcPtr cloudNans);
};

#endif