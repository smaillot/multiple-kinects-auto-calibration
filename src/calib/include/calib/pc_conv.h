#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

void convert(const sensor_msgs::PointCloud2& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output);

void convert(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PCLPointCloud2Ptr& output);
void convert(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output);
void convert(const pcl::PCLPointCloud2Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output);
void convert(const pcl::PCLPointCloud2Ptr& input, sensor_msgs::PointCloud2ConstPtr& output);
void convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PCLPointCloud2Ptr& output);
void convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, sensor_msgs::PointCloud2& output);