#include "geometry/PointCloud.h"

/**
 * @brief Line class constructor from pcl PointCloud.
 * 
 * @param cloud Input cloud.
 */
PointCloud::PointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::PCLPointCloud2* cloudPtr(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(cloud, *cloudPtr);
    this->cloud = cloudPtr;
}

/**
 * @brief Line class constructor from pcl PointCloud2.
 * 
 * @param cloud Input cloud.
 */
PointCloud::PointCloud(pcl::PCLPointCloud2& cloud)
{
    pcl::PCLPointCloud2* cloudPtr = new pcl::PCLPointCloud2(cloud);
    this->cloud = cloudPtr;
}

/**
 * @brief Line class constructor from sensor_msgs PointCloud2.
 * 
 * @param cloud Input cloud.
 */
PointCloud::PointCloud(sensor_msgs::PointCloud2& cloud)
{
    pcl::PCLPointCloud2* cloudPtr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(cloud, *cloudPtr);
    this->cloud = cloudPtr;
}