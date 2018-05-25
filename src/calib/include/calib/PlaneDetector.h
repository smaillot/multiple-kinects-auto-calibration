#ifndef DEF_PLANEDETECTOR
#define DEF_PLANEDETECTOR
 
#include <string>
#include <ros/console.h>
#include "Cloud.h"
#include "Preprocessing.h"
#include "PlaneDetector.h"
 
#include <shape_msgs/Plane.h>
#include <calib/Planes.h>
// #include <Eigen/Geometry>
// #include <Eigen/Dense>
 
#include <dynamic_reconfigure/server.h>
#include <calib/PlaneConfig.h>
 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <pcl_ros/transforms.h>
// #include <tf_conversions/tf_eigen.h>
 
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/vfh.h>
// #include <pcl/features/fpfh.h>
// #include <pcl/features/shot.h>
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/registration/correspondence_rejection_one_to_one.h>
 
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>
 
 
using namespace std;
 
struct param_plane_t
{
  int method;
  int n_planes;
  double th_dist;
  int max_it;
};
 
class PlaneDetector
{
private:
  ros::NodeHandle* node;
  tf::TransformListener *tf_listener;
  string name;

  ros::Subscriber sub;
  ros::Publisher pub_planes_col;
  ros::Publisher pub_planes;
  float subsize;

  pcl::SACSegmentation<Point> seg;
  pcl::ExtractIndices<Point> extract;
	pcl::VoxelGrid<Point> filter_voxel;

  param_plane_t param_plane;
  vector <Eigen::Vector4f> planes;
  
 
public:
  PlaneDetector(ros::NodeHandle* node, string name, string pub_name);

  void conf_callback(calib::PlaneConfig &config, uint32_t level);
  void update(const pcConstPtr& input);

  pcPtr subsample(const pcPtr& input, param_voxel_t params);
  void detect_plane(const pcPtr& input, param_plane_t params);
};

#endif