#include "Cloud.h"

using namespace std;
using namespace Eigen;

Cloud::Cloud(ros::NodeHandle* node, string sub_name, string pub_name)
{
    this->node = node;
	this->tf_listener = new tf::TransformListener; 
    this->sub = this->node->subscribe(sub_name, 1, &Cloud::update, this);
    this->pub_raw = this->node->advertise<sensor_msgs::PointCloud2>(pub_name, 1);
    this->pub_preproc = this->node->advertise<sensor_msgs::PointCloud2>(pub_name + "/preproc", 1);

    this->reset_params();
}

/*
* @brief Reset the parameters to default values.
*/
void Cloud::reset_params()
{
    this->param_voxel = {true, 0.02, 0.02, 0.02};
}

/*
* @brief Dynamic reconfiguer callback.
*/
void Cloud::conf_callback(calib::CloudConfig &config, uint32_t level)
{
// transform
    this->param_transform.tx = config.tx / 1000;
    this->param_transform.ty = config.ty / 1000;
    this->param_transform.tz = config.tz / 1000;
    this->param_transform.rx = config.rx / 180 * 3.14159;
    this->param_transform.ry = config.ry / 180 * 3.14159;
    this->param_transform.rz = config.rz / 180 * 3.14159;

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

// converters
    /*
    * @brief Convert point cloud between pcl and sensor_msg objects.
    *
    * @param input Input point cloud.
    * @param output Output point cloud.
    */
    void Cloud::convert(const pcl::PCLPointCloud2Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output)
    {
        pcl::fromPCLPointCloud2(*input, *output);    
    }

    /*
    * @brief Convert point cloud between pcl and sensor_msg objects.
    *
    * @param input Input point cloud.
    * @param output Output point cloud.
    * */
    void Cloud::convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PCLPointCloud2Ptr& output)
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
    void Cloud::convert(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PCLPointCloud2Ptr& output)
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
    void Cloud::convert(const pcl::PCLPointCloud2Ptr& input, sensor_msgs::PointCloud2ConstPtr& output)
    {
        sensor_msgs::PointCloud2* temp = new sensor_msgs::PointCloud2;
        pcl_conversions::fromPCL(*input, *temp);
        pcl_conversions::fromPCL(input->header, temp->header);
        output = sensor_msgs::PointCloud2ConstPtr(temp);
    }

// publishers
    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param msgPtr Pointer to the message or point cloud.
    */
    void Cloud::publish(ros::Publisher& pub, const sensor_msgs::PointCloud2ConstPtr& msgPtr, string frame)
    {
        sensor_msgs::PointCloud2 msg = *msgPtr; 
        pcl_ros::transformPointCloud(frame, msg, msg, *this->tf_listener);  

        if (msg.data.size() > 0)
        {
            pub.publish(msg);
            ROS_DEBUG_STREAM("Publishing\t" << msg.data.size() << " points on\t" << pub.getTopic());
        }
    }

    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param msgPtr Pointer to the message or point cloud.
    */
    void Cloud::publish(ros::Publisher& pub, const pcl::PCLPointCloud2Ptr& msgPtr, string frame)
    {
        sensor_msgs::PointCloud2ConstPtr outPtr;
        this->convert(msgPtr, outPtr);
        this->publish(pub, outPtr, frame);
    }


Eigen::Matrix4f Cloud::get_transform(param_transform_t params)
{
    Transform<float, 3, Eigen::Affine> t;
    t = Translation<float, 3>(params.tx, params.ty, params.tz);
    t.rotate(AngleAxis<float>(params.rx, Vector3f::UnitX()));
    t.rotate(AngleAxis<float>(params.ry, Vector3f::UnitY()));
    t.rotate(AngleAxis<float>(params.rz, Vector3f::UnitZ()));
    return t.matrix();
}

/*
* @brief Subscriber callback containing processing loop
*/
void Cloud::update(const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2* msg(new sensor_msgs::PointCloud2(*input)); 
    pcl_ros::transformPointCloud("world", *msg, *msg, *this->tf_listener);  
    pcl_ros::transformPointCloud(this->get_transform(this->param_transform), *msg, *msg);  
    sensor_msgs::PointCloud2ConstPtr msg_ptr(msg);
    
    if (this->pub_raw.getTopic() != this->sub.getTopic())     
    {
        this->publish(this->pub_raw, msg_ptr, "world");
    }

    pcl::PCLPointCloud2Ptr cloud2Ptr;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

    this->convert(msg_ptr, cloud2Ptr);

    cloud2Ptr = this->subsample(cloud2Ptr, this->param_voxel);
    cloud2Ptr = this->cut(cloud2Ptr, this->param_cut);

    this->publish(this->pub_preproc, cloud2Ptr, "cam_center");

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
    // this->convert(cloud2ptr, cloudPtr);
    ROS_DEBUG_STREAM("End processing loop for " << this->sub.getTopic() << "\n");
}

/*
 * @brief Apply a voxel filter to the point cloud. 
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcl::PCLPointCloud2Ptr Cloud::subsample(const pcl::PCLPointCloud2Ptr& input, param_voxel_t params)
{
    if (params.enable)
    {
        ROS_DEBUG_STREAM("Applying voxel filter with parameters: (" << params.x << ", " << params.y << ", " << params.z << ")");
        this->filter_voxel.setInputCloud(input);
        this->filter_voxel.setLeafSize(params.x, params.y, params.z);
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
        this->filter_voxel.filter(*cloud);
        pcl::PCLPointCloud2Ptr output(cloud);
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
pcl::PCLPointCloud2Ptr Cloud::cut(pcl::PCLPointCloud2Ptr input, param_cut_t params)
{
    if (this->param_cut.x.enable || this->param_cut.y.enable || this->param_cut.z.enable)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>); 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>); 
        this->convert(input, temp);
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
        pcl::PCLPointCloud2Ptr output;
        this->convert(temp, output);
        return output;
    }
    else
    {
        return input;
    }
}