#include "geometry/PointCloud.h"

using namespace geometry;


/**
 * @brief Default constructor.
 */
PointCloud::PointCloud()
{

}

/**
 * @brief Initialize object.
 * 
 * @param listener TransformListener object.
 */
void PointCloud::init(tf::TransformListener* listener)
{
    this->tf_listener = listener;
}

/**
 * @brief Point cloud getter.
 */
sensor_msgs::PointCloud2* PointCloud::get_pc()
{
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2* ptr = new sensor_msgs::PointCloud2(output);
    pcl_conversions::fromPCL(*this->cloud, *ptr);
    return ptr;
}

/**
 * @brief Update point cloud from topic.
 * 
 * @param cloud Input cloud message.
 */
void PointCloud::update(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    sensor_msgs::PointCloud2 msg = *cloud;
    pcl::PCLPointCloud2* cloudPtr(new pcl::PCLPointCloud2);
    pcl_ros::transformPointCloud(REFERENCE_FRAME, msg, msg, *this->tf_listener);
    pcl_conversions::toPCL(msg, *cloudPtr);
    this->cloud = cloudPtr;
}

/**
 * @brief Subsample evenly the point cloud.
 */
void PointCloud::subsample()
{
    if (subsize > 0)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloudPtr);
        sor.setLeafSize(this->subsize, this->subsize, this->subsize);
        sor.filter(*this->cloud);
    }
}

/**
 * @brief Perform radius filtering on the point cloud
 */
void PointCloud::radius_filter()
{
    if (this->filtering)
    {
        // filtering
        pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;

        // build the filter
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        outrem.setInputCloud(cloudPtr);
        outrem.setRadiusSearch(this->filter_radius);
        outrem.setMinNeighborsInRadius(this->filter_min_neighbors);

        // apply filter
        outrem.filter(*this->cloud);
    }
    else
    {
        // filtering disableds
    }
}

/**
 * @brief Perform cutting of the point cloud.
 */
void PointCloud::cut()
{
    if (this->cutting.x.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudPtr);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (this->cutting.x.bounds[0], this->cutting.x.bounds[1]);
        pass.filter(*this->cloud);
    }
    if (this->cutting.y.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudPtr);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (this->cutting.y.bounds[0], this->cutting.y.bounds[1]);
        pass.filter(*this->cloud);
    }
    if (this->cutting.z.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudPtr);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (this->cutting.z.bounds[0], this->cutting.z.bounds[1]);
        pass.filter(*this->cloud);
    }
}