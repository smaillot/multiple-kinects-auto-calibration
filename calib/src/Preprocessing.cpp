#include "Preprocessing.h"

using namespace std;
using namespace Eigen;

Preprocessing::Preprocessing(ros::NodeHandle* node, Cloud* cloud)
{
    this->name = cloud->name;
	this->frame = cloud->frame;
	this->frame_pub = cloud->frame_pub;
    string topic = "/calib/clouds/" + this->name;
    this->node = node;
	this->tf_listener = new tf::TransformListener; 
    this->frame = cloud->frame;
    this->frame_pub = cloud->frame_pub;
    this->sub = this->node->subscribe(topic, 1, &Preprocessing::update, this);
    this->pub_preproc = this->node->advertise<pc_t>(topic + "/preproc", 1);
}

/*
* @brief Dynamic reconfiguer callback.
*/
void Preprocessing::conf_callback(calib::PreprocessingConfig &config, uint32_t level)
{
    this->frame = config.frame;
    // subsampling
    this->param_voxel.enable = config.enable;
    this->param_voxel.x = config.size / 1000;
    this->param_voxel.y = config.size / 1000;
    this->param_voxel.z = config.size / 1000;
    // cutting
    this->param_cut.x.enable = config.x_enable;
    this->param_cut.x.bounds[0] = config.x_min / 1000;
    this->param_cut.x.bounds[1] = config.x_max / 1000;
    this->param_cut.y.enable = config.y_enable;
    this->param_cut.y.bounds[0] = config.y_min / 1000;
    this->param_cut.y.bounds[1] = config.y_max / 1000;
    this->param_cut.z.enable = config.z_enable;
    this->param_cut.z.bounds[0] = config.z_min / 1000;
    this->param_cut.z.bounds[1] = config.z_max / 1000;
}

// publishers
    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param cloud Point cloud.
    */
    void Preprocessing::publish(ros::Publisher& pub, pc_t& cloud)
    {
        pc_msg_t msg;
        pcl::toROSMsg(cloud, msg);
        this->publish(pub, msg);
    }

    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param cloudPtr Pointer to the point cloud.
    */
    void Preprocessing::publish(ros::Publisher& pub, const pcConstPtr& cloudPtr)
    {
        pc_t cloud = *cloudPtr; 
        this->publish(pub, cloud);
    }

    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param input Point cloud pointer.
    */
    void Preprocessing::publish(ros::Publisher& pub, const pc_msg_t& input)
    {
        pc_msg_t msg = input;
        pcl_ros::transformPointCloud(this->frame_pub, msg, msg, *this->tf_listener);  

        if (msg.data.size() > 0)
        {
            pub.publish(msg);
        }
    }

/*
 * @brief Apply a voxel filter to the point cloud. 
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr Preprocessing::subsample(const pcPtr& input, param_voxel_t params)
{
    if (params.enable && input->points.size() > 0)
    {
        this->filter_voxel.setInputCloud(input);
        this->filter_voxel.setLeafSize(params.x, params.y, params.z);
        pc_t* cloud = new pc_t; 
        this->filter_voxel.filter(*cloud);
        pcPtr output(cloud);
        if (output->points.size() > 0)
        {
            return output;
        }
        else
        {
            return input;
        }
    }
    else
    {
        return input;
    }
}

/**
 * @brief Perform cutting of the point cloud.
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr Preprocessing::cut(pcPtr input, param_cut_t params)
{
    ROS_DEBUG("Cutting point cloud...");
    if (params.x.enable || params.y.enable || params.z.enable)
    {
        ROS_DEBUG("Cutting request found.");
        pcPtr temp(new pc_t(*input)); 
        pcPtr filtered(new pc_t); 
        if (params.x.enable) 
        {
            ROS_DEBUG("Cutting in X.");
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("x"); 
            this->filter_cut.setFilterLimits(params.x.bounds[0], params.x.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (params.y.enable) 
        { 
            ROS_DEBUG("Cutting in Y.");
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("y"); 
            this->filter_cut.setFilterLimits(params.y.bounds[0], params.y.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (params.z.enable) 
        { 
            ROS_DEBUG("Cutting in Z.");
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("z"); 
            this->filter_cut.setFilterLimits(params.z.bounds[0], params.z.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered;
        }
        if (temp->points.size() > 0)
        {
            return temp;
        }
        else
        {
            ROS_WARN("Point cloud empty after cutting, cancelling...");
            return input;
        }
    }
    else
    {
        ROS_DEBUG("No cutting request detected.");
        return input;
    }
}

/*
* @brief Subscriber callback containing processing loop
*/
void Preprocessing::update(const pcConstPtr& input)
{
    pc_t* cloud(new pc_t(*input));
    pcPtr cloudPtr(cloud);
    pcl_ros::transformPointCloud(this->frame, *cloud, *cloud, *this->tf_listener); 
    cloudPtr = this->cut(cloudPtr, this->param_cut);
    cloudPtr = this->subsample(cloudPtr, this->param_voxel);
    this->publish(this->pub_preproc, cloudPtr);
}