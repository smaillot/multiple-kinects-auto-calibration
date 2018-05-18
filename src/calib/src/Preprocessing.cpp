#include "Preprocessing.h"

using namespace std;
using namespace Eigen;

Preprocessing::Preprocessing(ros::NodeHandle* node, Cloud* cloud)
{
    // this->name = pub_name;
    // string topic = "/calib/clouds/" + pub_name;
    // ROS_DEBUG_STREAM("Init Cloud instance " << pub_name << ": " << sub_name);
    // ROS_DEBUG_STREAM("\tpublish topic: " << topic);
    // this->node = node;
	// this->tf_listener = new tf::TransformListener; 
    // this->frame = "world";
    // this->sub = this->node->subscribe(sub_name, 1, &Cloud::update, this);
    // this->pub_raw = this->node->advertise<pc_t>(topic, 1);

    // this->node_preproc = node_preproc;
    // this->pub_preproc = this->node->advertise<pc_msg_t>("/calib/clouds/" + pub_name + "/preproc", 1);
    // this->pub_planes_pc_col = this->node->advertise<pc_msg_t>("/calib/clouds/" + pub_name + "/planes_colored", 1);
    // this->pub_planes_pc = this->node->advertise<calib::PlaneClouds>("/calib/clouds/" + pub_name + "/planes", 1);
    // this->pub_planes = this->node->advertise<calib::Planes>("/calib/planes/" + pub_name, 1);
}

/*
* @brief Dynamic reconfiguer callback.
*/
void Preprocessing::conf_callback(calib::PreprocessingConfig &config, uint32_t level)
{

    // subsampling
    this->param_voxel.enable = config.enable;
	if (config.equal)
    {
        this->param_voxel.x = config.x / 1000;
        this->param_voxel.y = config.x / 1000;
        this->param_voxel.z = config.x / 1000;
    }
    else
    {
        this->param_voxel.x = config.x / 1000;
        this->param_voxel.y = config.y / 1000;
        this->param_voxel.z = config.z / 1000;
    }
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

/*
 * @brief Apply a voxel filter to the point cloud. 
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr Preprocessing::subsample(const pcPtr& input, param_voxel_t params)
{
    if (params.enable)
    {
        this->filter_voxel.setInputCloud(input);
        this->filter_voxel.setLeafSize(params.x, params.y, params.z);
        pc_t* cloud = new pc_t; 
        this->filter_voxel.filter(*cloud);
        pcPtr output(cloud);
        return output;
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
    if (this->param_cut.x.enable || this->param_cut.y.enable || this->param_cut.z.enable)
    {
        pcPtr temp(new pc_t(*input)); 
        pcPtr filtered(new pc_t); 
        if (this->param_cut.x.enable) 
        {
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("x"); 
            this->filter_cut.setFilterLimits(this->param_cut.x.bounds[0], this->param_cut.x.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (this->param_cut.y.enable) 
        { 
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("y"); 
            this->filter_cut.setFilterLimits(this->param_cut.y.bounds[0], this->param_cut.y.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (this->param_cut.z.enable) 
        { 
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("z"); 
            this->filter_cut.setFilterLimits(this->param_cut.z.bounds[0], this->param_cut.z.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered;
        }
        return temp;
    }
    else
    {
        return input;
    }
}

/*
* @brief Subscriber callback containing processing loop
*/
void Preprocessing::update(const pcConstPtr& input)
{
    Cloud::update(input); 
    pcPtr cloudPtr = this->cloud;
    cloudPtr = this->cut(cloudPtr, this->param_cut);
    cloudPtr = this->subsample(cloudPtr, this->param_voxel);
    this->publish(this->pub_preproc, cloudPtr);
}