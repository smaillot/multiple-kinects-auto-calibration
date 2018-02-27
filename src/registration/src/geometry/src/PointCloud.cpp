#include "geometry/PointCloud.h"

using namespace geometry;


/**
 * @brief Default constructor.
 */
PointCloud::PointCloud(ros::NodeHandle nh, std::string topic_name, std::string pub_name)
{
    this->node = nh;
	this->tf_listener = new tf::TransformListener; 

    this->sub_name = topic_name;
    this->pub_name = pub_name;

    subsampling_params_t subsampling_params = {false, 0.02, 0.02, 0.02};
    cutting_params_t cutting_params = {{false, -1.0, 1.0}, {false, -1.0, 1.0}, {false, -1.0, 1.0}};
    radius_filtering_params_t radius_filtering_params = {false, 0.1, 50};

    this->subsampling_params = subsampling_params; 
    this->cutting_params = cutting_params;
    this->radius_filtering_params = radius_filtering_params;

    this->pc_sub = this->node.subscribe(this->sub_name, 1, &PointCloud::update, this);
    this->pc_pub = this->node.advertise<sensor_msgs::PointCloud2>(this->pub_name, 1);
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
 * @brief Subsampling parameters setter.
 */
void PointCloud::set_subsampling_params(subsampling_params_t subsamples_params)
{
    this->subsampling_params = subsamples_params;
}

/**
 * @brief Cutting parameters setter.
 */
void PointCloud::set_cutting_params(cutting_params_t cutting_params)
{
    this->cutting_params = cutting_params;
}

/**
 * @brief Radius filtering parameters setter.
 */
void PointCloud::set_radius_filtering_params(radius_filtering_params_t radius_filtering_params)
{
    this->radius_filtering_params = radius_filtering_params;
}

/**
 * @brief Update point cloud from topic.
 * 
 * @param cloud Input cloud message.
 */
void PointCloud::update(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    ROS_DEBUG("Updating PC object");
    // init variables
        sensor_msgs::PointCloud2 msg = *cloud;
        pcl::PCLPointCloud2* cloudPtr(new pcl::PCLPointCloud2);
        ros::Time t = ros::Time(0);
        pcl_ros::transformPointCloud(REFERENCE_FRAME, msg, msg, *this->tf_listener);
        pcl_conversions::toPCL(msg, *cloudPtr);
        this->cloud = cloudPtr;

    /* process cloud */
    
        this->subsample();
        // this->cut();
        // this->radius_filter();
    
    /*****************/

    if (this->cloud->data.size() > 0)
    {
        // publish
            sensor_msgs::PointCloud2* msg_pub;
            msg_pub = this->get_pc();
            this->pc_pub.publish(*msg_pub);
    }
    else
    {
        ROS_DEBUG("Can't publish, point cloud is empty");
    }
}

/**
 * @brief Subsample evenly the point cloud.
 */
void PointCloud::subsample()
{
    if (this->subsampling_params.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(new pcl::PCLPointCloud2(*this->cloud));
        this->filter_voxel.setInputCloud(cloudPtr);
        this->filter_voxel.setLeafSize(this->subsampling_params.x, this->subsampling_params.y, this->subsampling_params.z);
        this->filter_voxel.filter(*this->cloud);
    }
}

/**
 * @brief Perform cutting of the point cloud.
 */
void PointCloud::cut()
{
    if (this->cutting_params.x.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(new pcl::PCLPointCloud2(*this->cloud));
        this->filter_cut.setInputCloud(cloudPtr);
        this->filter_cut.setFilterFieldName("x");
        this->filter_cut.setFilterLimits(this->cutting_params.x.bounds[0], this->cutting_params.x.bounds[1]);
        this->filter_cut.filter(*this->cloud);
    }
    if (this->cutting_params.y.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(new pcl::PCLPointCloud2(*this->cloud));
        this->filter_cut.setInputCloud(cloudPtr);
        this->filter_cut.setFilterFieldName("y");
        this->filter_cut.setFilterLimits(this->cutting_params.y.bounds[0], this->cutting_params.y.bounds[1]);
        this->filter_cut.filter(*this->cloud);
    }
    if (this->cutting_params.z.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(new pcl::PCLPointCloud2(*this->cloud));
        this->filter_cut.setInputCloud(cloudPtr);
        this->filter_cut.setFilterFieldName("z");
        this->filter_cut.setFilterLimits(this->cutting_params.z.bounds[0], this->cutting_params.z.bounds[1]);
        this->filter_cut.filter(*this->cloud);
    }
}

/**
 * @brief Perform radius filtering on the point cloud
 */
void PointCloud::radius_filter()
{
    if (this->radius_filtering_params.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(new pcl::PCLPointCloud2(*this->cloud));
        filter_radius.setInputCloud(cloudPtr);
        filter_radius.setRadiusSearch(this->radius_filtering_params.radius);
        filter_radius.setMinNeighborsInRadius(this->radius_filtering_params.min_neighbors);
        filter_radius.filter(*this->cloud);
    }
    else
    {
        // filtering disableds
    }
}