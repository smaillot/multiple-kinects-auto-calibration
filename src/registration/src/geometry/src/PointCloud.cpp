#include "geometry/PointCloud.h"

using namespace geometry;
using namespace std;


/**
 * @brief Default constructor.
 */
PointCloud::PointCloud(ros::NodeHandle nh, std::string topic_name, std::string pub_name)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    this->node = nh;
	this->tf_listener = new tf::TransformListener; 
    this->frame = "";

    this->sub_name = topic_name;
    this->pub_name = pub_name;

    subsampling_params_t subsampling_params = {false, 0.02, 0.02, 0.02};
    cutting_params_t cutting_params = {{false, -1.0, 1.0}, {false, -1.0, 1.0}, {false, -1.0, 1.0}};
    radius_filtering_params_t radius_filtering_params = {false, 0.1, 50};
    outliers_removal_params_t outliers_removal_params = {false, 50, 1.0};

    this->subsampling_params = subsampling_params; 
    this->cutting_params = cutting_params;
    this->radius_filtering_params = radius_filtering_params;
    this->outliers_removal_params = outliers_removal_params;

    this->pc_sub = this->node.subscribe(this->sub_name, 1, &PointCloud::update, this);
    this->pc_pub = this->node.advertise<sensor_msgs::PointCloud2>(this->pub_name + "/preprocessed", 1);
    this->pc_pub_raw = this->node.advertise<sensor_msgs::PointCloud2>(this->pub_name, 1);
    ROS_INFO_STREAM("Start publishing raw and preprocessed data from " + sub_name);
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

void PointCloud::set_outliers_removal_params(outliers_removal_params_t outliers_removal_params)
{
    this->outliers_removal_params = outliers_removal_params;
}

void PointCloud::change_frame(std::string frame)
{
    this->frame = frame;
}

/**
 * @brief Update point cloud from topic.
 * 
 * @param cloud Input cloud message.
 */
void PointCloud::update(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_DEBUG("Updating PC object");
    std::string current_frame = cloud_msg->header.frame_id;

    // convert message

        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        
        sensor_msgs::PointCloud2 msg = *cloud_msg; 
        //ros::Time t = ros::Time(0);  
        pcl_ros::transformPointCloud("cam_center", msg, msg, *this->tf_listener);  

        if (msg.data.size() > 0 && this->sub_name != this->pub_name)
        {
            // publish
                this->pc_pub_raw.publish(msg);
                ROS_DEBUG_STREAM("Publish raw point cloud on " + this->pub_name + " (" + patch::to_string(cloud_msg->data.size()) + " points)");
        }
        else
        {
            ROS_DEBUG("Can't publish, raw point cloud is empty or already published");
        }
        
        pcl_conversions::toPCL(msg, *cloud);

    /* process cloud */
    
        cloudPtr = this->subsample(cloudPtr);
        cloudPtr = this->outliers_removal(cloudPtr);
        cloudPtr = this->cut(cloudPtr);

        pcl_conversions::fromPCL(*cloudPtr, msg);
    
    /*****************/

    if (msg.data.size() > 0)
    {
        // publish
            this->pc_pub.publish(msg);
            ROS_DEBUG_STREAM("Publish preprocessed point cloud on " + this->pub_name + " (" + patch::to_string(msg.data.size()) + " points)");
    }
    else
    {
        ROS_DEBUG("Can't publish, preprocessed point cloud is empty");
    }
}

/**
 * @brief Subsample evenly the point cloud.
 */
pcl::PCLPointCloud2ConstPtr PointCloud::subsample(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
    if (this->subsampling_params.enable)
    {
        this->filter_voxel.setInputCloud(cloudPtr);
        this->filter_voxel.setLeafSize(this->subsampling_params.x, this->subsampling_params.y, this->subsampling_params.z);
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
        this->filter_voxel.filter(*cloud);
        pcl::PCLPointCloud2ConstPtr temp(cloud);
        cloudPtr = temp;
    }
    return cloudPtr;
}

/**
 * @brief Perform cutting of the point cloud.
 */
pcl::PCLPointCloud2ConstPtr PointCloud::cut(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
    if (this->cutting_params.x.enable || this->cutting_params.y.enable || this->cutting_params.z.enable)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(*cloudPtr, *temp_cloud);
        if (this->cutting_params.x.enable) 
        {
            this->filter_cut.setInputCloud(temp_cloud); 
            this->filter_cut.setFilterFieldName("x"); 
            this->filter_cut.setFilterLimits(this->cutting_params.x.bounds[0], this->cutting_params.x.bounds[1]); 
            this->filter_cut.filter(*filtered); 
            temp_cloud = filtered;
        } 
        if (this->cutting_params.y.enable) 
        { 
            this->filter_cut.setInputCloud(temp_cloud); 
            this->filter_cut.setFilterFieldName("y"); 
            this->filter_cut.setFilterLimits(this->cutting_params.y.bounds[0], this->cutting_params.y.bounds[1]); 
            this->filter_cut.filter(*filtered); 
            temp_cloud = filtered;
        } 
        if (this->cutting_params.z.enable) 
        { 
            this->filter_cut.setInputCloud(temp_cloud); 
            this->filter_cut.setFilterFieldName("z"); 
            this->filter_cut.setFilterLimits(this->cutting_params.z.bounds[0], this->cutting_params.z.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp_cloud = filtered;
        }
        pcl::PCLPointCloud2* output(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*filtered, *output);
        pcl::PCLPointCloud2ConstPtr temp(output);
        cloudPtr = temp;
    }
    return cloudPtr;
}

/**
 * @brief Perform radius filtering on the point cloud
 */
pcl::PCLPointCloud2ConstPtr PointCloud::radius_filter(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
    if (this->radius_filtering_params.enable)
    {
        pcl::PCLPointCloud2 filtered;

        this->filter_radius.setInputCloud(cloudPtr);
        this->filter_radius.setRadiusSearch(this->radius_filtering_params.radius);
        this->filter_radius.setMinNeighborsInRadius(this->radius_filtering_params.min_neighbors);
        this->filter_radius.filter(filtered);

        if (filtered.data.size() > 0)
        {
            pcl::PCLPointCloud2ConstPtr temp(&filtered);
            cloudPtr = temp;
        }
    }
    else
    {
        // filtering disabled
    }
    return cloudPtr;
}

pcl::PCLPointCloud2ConstPtr PointCloud::outliers_removal(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
    if (this->outliers_removal_params.enable)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(*cloudPtr, *temp_cloud);

        this->sor.setInputCloud(temp_cloud);
        this->sor.setMeanK (this->outliers_removal_params.meank);
        this->sor.setStddevMulThresh (this->outliers_removal_params.std_mul);
        this->sor.filter(*filtered);

        pcl::PCLPointCloud2* output(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*filtered, *output);
        pcl::PCLPointCloud2ConstPtr temp(output);
        cloudPtr = temp;
    }
    return cloudPtr;
}

// void PointCloud::transform()
// {
//     if (this->tf_listener->waitForTransform(this->cloud->header.frame_id, this->frame, ros::Time::now(), ros::Duration(1.0)))
//     {
//         sensor_msgs::PointCloud2* input = this->get_pc();
//         sensor_msgs::PointCloud2* msg;
//         pcl_ros::transformPointCloud(this->frame, *input, *msg, *this->tf_listener);
//         pcl::PCLPointCloud2* output;
//         pcl_conversions::toPCL(*msg, *output);
//         this->cloud = output;
//     }
//     else
//     {
//         ROS_ERROR_STREAM("No tf received from " + this->cloud->header.frame_id + " to " + this->frame + " in 1s. Abort point cloud transform.");
//     }
// }