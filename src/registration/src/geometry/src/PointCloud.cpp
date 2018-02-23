#include "geometry/PointCloud.h"

using namespace geometry;


/**
 * @brief Default constructor.
 */
PointCloud::PointCloud(ros::NodeHandle nh, std::string topic_name, std::string pub_name)
{
    this->node = nh;

    this->sub_name = topic_name;
    this->pub_name = pub_name;

    this->tf_listener = new tf::TransformListener;
    dynamic_reconfigure::Server<SubSamplingConfig>::CallbackType subsampling_cb; // pb here
    subsampling_cb = boost::bind(&PointCloud::subsampling_conf_callback, this, _1, _2);
    this->subsampling_config_srv.setCallback(subsampling_cb);

    boost::recursive_mutex::scoped_lock dyn_reconf_lock(this->subsampling_config_mutex);  
    dyn_reconf_lock.unlock();  

    subsampling_config_cb = boost::bind(&PointCloud::subsampling_conf_callback, this, _1, _2);  
    subsampling_config_srv.setCallback(subsampling_config_cb);  

    // Config config;
    // subsampling_config_srv.getConfigDefault(config);
    // subsampling_config_srv.updateConfig(config);

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
 * @brief Callback for subsampling dynamic reconfigure.
 */
void PointCloud::subsampling_conf_callback(geometry::SubSamplingConfig &config, uint32_t level)
{

    this->subsampling_param.x = config.size / 1000;
    this->subsampling_param.y = config.size / 1000;
    this->subsampling_param.z = config.size / 1000;
    
    // this->subsampling_param.x = config.size_x;
    // this->subsampling_param.y = config.size_y;
    // this->subsampling_param.z = config.size_z;

    this->subsampling_param.enable = config.enable;

    ROS_DEBUG("Subsampling config updated");
}

/**
 * @brief Update point cloud from topic.
 * 
 * @param cloud Input cloud message.
 */
void PointCloud::update(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    ROS_DEBUG("Updating PC object");
    sensor_msgs::PointCloud2 msg = *cloud;
    pcl::PCLPointCloud2* cloudPtr(new pcl::PCLPointCloud2);
    ros::Time t = ros::Time(0);
    pcl_ros::transformPointCloud(REFERENCE_FRAME, msg, msg, *this->tf_listener);
    pcl_conversions::toPCL(msg, *cloudPtr);
    this->cloud = cloudPtr;

    // process cloud
    //
    this->subsample();
    //////////

    sensor_msgs::PointCloud2* msg_pub;
    msg_pub = this->get_pc();
    this->pc_pub.publish(*msg_pub);
}

/**
 * @brief Subsample evenly the point cloud.
 */
void PointCloud::subsample()
{
    if (this->subsampling_param.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        this->voxel_grid.setInputCloud(cloudPtr);
        this->voxel_grid.setLeafSize(this->subsampling_param.x, this->subsampling_param.y, this->subsampling_param.z);
        this->voxel_grid.filter(*this->cloud);
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
    if (this->cutting_param.x.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudPtr);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (this->cutting_param.x.bounds[0], this->cutting_param.x.bounds[1]);
        pass.filter(*this->cloud);
    }
    if (this->cutting_param.y.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudPtr);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (this->cutting_param.y.bounds[0], this->cutting_param.y.bounds[1]);
        pass.filter(*this->cloud);
    }
    if (this->cutting_param.z.enable)
    {
        pcl::PCLPointCloud2ConstPtr cloudPtr(this->cloud);
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloudPtr);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (this->cutting_param.z.bounds[0], this->cutting_param.z.bounds[1]);
        pass.filter(*this->cloud);
    }
}