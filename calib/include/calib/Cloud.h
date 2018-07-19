#ifndef DEF_CLOUD
#define DEF_CLOUD

#include <string>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <calib/CloudConfig.h>

#include <shape_msgs/Plane.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
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
typedef pc_msg_t::ConstPtr pc_msgConstPtr;

/**
*	@brief Structure containing point cloud rigid transform parameters
*/
struct param_transform_t
{
	float tx;
	float ty;
	float tz;
	float rx;
	float ry;
	float rz;
};

/**
*	@brief Convert a pointcloud message into pcl::PointCloud::Ptr
*/
void MSGtoPCL(const pc_msg_t& msg, pcPtr& cloud);
/**
*	@brief Set RGB color of all point cloud points
*	@param input The input cloud pointer
*	@param ratio The number between 0 and 1 corresponding to the desired color (0 and 1 are red)
*/
void colorize(pcPtr input, float ratio);

/**
*	@brief Point cloud class 
*	@section DESCRIPTION
*	Point cloud class using input 3D sensor data to publish a topic inside calib namespace.
*	It allows to trasform the input cloud to simulate or compensate camera position error manualy.
*/
class Cloud
{
	friend class Preprocessing;
private:
	param_transform_t param_transform;	///< Parameters of transform
	ros::NodeHandle* node;
	tf::TransformListener *tf_listener;
	string frame;
	string frame_pub;
	string name;
	bool icp;	///< If an ICP registration tf is published for this node

	ros::Subscriber sub;
	ros::Publisher pub_raw;

	pcPtr cloud;

public:
	Cloud();
	Cloud(ros::NodeHandle* node, string sub_name, string pub_name, bool icp);

	void conf_callback(calib::CloudConfig &config, uint32_t level);
	/**
	*	@brief Subscriber callback containing processing loop
	*	@param input Point cloud pointer
	*/
	void update(const pcConstPtr& input);

	/**
    *	@brief Publish point cloud.
    *
    *	@param pub Ros message publisher.
    *	@param cloud Point cloud.
    */
	void publish(ros::Publisher& pub, pc_t& msg);
	void publish(ros::Publisher& pub, const pcConstPtr& msg_ptr);
	void publish(ros::Publisher& pub, const pc_msg_t& msg);

	/**
	*	@brief Create tf from transform parameters
	*	@param params Transform parameters
	*	@param rot Include rotation part
	*	@param tr Include translation part
	*	@return tf rigid transform
	*/
	tf::Transform get_transform(param_transform_t params, bool rot, bool tr);
	/**
	*	@brief Remove NaN values from point cloud
	*	@param cloudNans Input point cloud
	*	@return Output point cloud
	*/
	pcPtr remove_nans(pcPtr cloudNans);
};

#endif
