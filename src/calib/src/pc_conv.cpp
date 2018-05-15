#include "calib/pc_conv.h"

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const pcl::PCLPointCloud2Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output)
{
    pcl::fromPCLPointCloud2(*input, *output);    
}

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
* */
void convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PCLPointCloud2Ptr& output)
{
    pcl::PCLPointCloud2* temp(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*input, *temp);
    output = pcl::PCLPointCloud2Ptr(temp);
}

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PCLPointCloud2Ptr& output)
{
    pcl::PCLPointCloud2* temp = new pcl::PCLPointCloud2; 
    pcl_conversions::toPCL(*input, *temp);
    pcl_conversions::toPCL(input->header, temp->header);
    output = pcl::PCLPointCloud2Ptr(temp); 
} 

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const pcl::PCLPointCloud2Ptr& input, sensor_msgs::PointCloud2ConstPtr& output)
{
    sensor_msgs::PointCloud2* temp = new sensor_msgs::PointCloud2;
    pcl_conversions::fromPCL(*input, *temp);
    pcl_conversions::fromPCL(input->header, temp->header);
    output = sensor_msgs::PointCloud2ConstPtr(temp);
}

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, sensor_msgs::PointCloud2& output)
{
    pcl::PCLPointCloud2Ptr temp;
    sensor_msgs::PointCloud2ConstPtr temp2;
    convert(input, temp);
    convert(temp, temp2);
    output = *temp2;
}

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output)
{
    pcl::PCLPointCloud2Ptr temp;
    convert(input, temp);
    convert(temp, output);
} 