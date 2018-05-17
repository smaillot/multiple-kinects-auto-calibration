#include "Cloud.h"

using namespace std;
using namespace Eigen;

void MSGtoPCL(const pc_msg_t& msg, pcPtr& cloud)
{
    if (msg.data.size() > 0)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcPtr temp(new pc_t);
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp);
        cloud = temp;
    }
}

Cloud::Cloud(ros::NodeHandle* node, string sub_name, string pub_name)
{
    this->node = node;
	this->tf_listener = new tf::TransformListener; 
    this->sub = this->node->subscribe(sub_name, 1, &Cloud::update, this);
    this->pub_raw = this->node->advertise<pc_t>("/calib/clouds/" + pub_name, 1);
    this->pub_preproc = this->node->advertise<pc_msg_t>("/calib/clouds/" + pub_name + "/preproc", 1);
    this->pub_planes_pc_col = this->node->advertise<pc_msg_t>("/calib/clouds/" + pub_name + "/planes_colored", 1);
    this->pub_planes_pc = this->node->advertise<calib::PlaneClouds>("/calib/clouds/" + pub_name + "/planes", 1);
    this->pub_planes = this->node->advertise<calib::Planes>("/calib/planes/" + pub_name, 1);

    this->seg.setOptimizeCoefficients(true);
    this->seg.setModelType(pcl::SACMODEL_PLANE);

    this->reset_params();
}

/*
* @brief Reset the parameters to default values.
*/
void Cloud::reset_params()
{
    // this->param_voxel = {true, 0.02, 0.02, 0.02};
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
    
    // plane detection
    this->param_plane.method = config.method;
    this->param_plane.n_planes = config.n_planes;
    this->param_plane.th_dist = config.th_dist / 1000;
    this->param_plane.max_it = config.max_it;
}

// publishers
    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param cloud Point cloud.
    */
    void Cloud::publish(ros::Publisher& pub, pc_t& cloud, string frame)
    {
        pc_msg_t msg;
        pcl::toROSMsg(cloud, msg);
        this->publish(pub, msg, frame);
    }

    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param cloudPtr Pointer to the point cloud.
    */
    void Cloud::publish(ros::Publisher& pub, const pcConstPtr& cloudPtr, string frame)
    {
        pc_t cloud = *cloudPtr; 
        this->publish(pub, cloud, frame);
    }

    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param input Point cloud pointer.
    */
    void Cloud::publish(ros::Publisher& pub, const pc_msg_t& input, string frame)
    {
        pc_msg_t msg = input;
        pcl_ros::transformPointCloud(frame, msg, msg, *this->tf_listener);  

        if (msg.data.size() > 0)
        {
            pub.publish(msg);
        }
    }

tf::Transform Cloud::get_transform(param_transform_t params, bool rot, bool tr)
{
    Eigen::Affine3d t = Affine3d::Identity();
    tf::Transform transform;
    if (tr)
    {
        t.translation() << params.tx, params.ty, params.tz;
    }
    if (rot)
    {
        t.rotate(AngleAxisd(params.rx, Vector3d::UnitX()));
        t.rotate(AngleAxisd(params.ry, Vector3d::UnitY()));
        t.rotate(AngleAxisd(params.rz, Vector3d::UnitZ()));
    }
    tf::transformEigenToTF(t, transform);
    return transform;
}

/*
* @brief Subscriber callback containing processing loop
*/
void Cloud::update(const pcConstPtr& input)
{
    pc_t* cloud(new pc_t(*input));
    pcPtr cloudPtr(cloud); 
    pcl_ros::transformPointCloud("world", *cloud, *cloud, *this->tf_listener);  
    pcl_ros::transformPointCloud(*cloud, *cloud, this->get_transform(this->param_transform, true, true));  
    cloudPtr = this->cut(cloudPtr, this->param_cut);
    if (this->pub_raw.getTopic() != this->sub.getTopic())     
    {
        this->publish(this->pub_raw, cloudPtr, "world");
    }
    cloudPtr = this->subsample(cloudPtr, this->param_voxel);
    this->publish(this->pub_preproc, cloudPtr, "world");
    this->planes = this->detect_plane(cloudPtr, this->param_plane);
}

/*
 * @brief Apply a voxel filter to the point cloud. 
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr Cloud::subsample(const pcPtr& input, param_voxel_t params)
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
pcPtr Cloud::cut(pcPtr input, param_cut_t params)
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
* @brief Detect planes in a point cloud and publish a segmented point cloud.
*
* @param input Input point cloud.
* @param params Plane detection parameters.
*/
vector <Eigen::Vector4f> Cloud::detect_plane(const pcPtr& input, param_plane_t params)
{
    vector <Eigen::Vector4f> planes;
    if (params.method >= 0 && params.n_planes && params.th_dist >= 0 && params.max_it > 0)
    {
        seg.setMethodType(params.method);
        this->seg.setMaxIterations(params.max_it);
        this->seg.setDistanceThreshold (params.th_dist);
        pcPtr remaining(new pc_t);
        remaining = input;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        calib::PlaneClouds planes_sep;
        calib::Planes coef_msg;
        shape_msgs::Plane plane_eq;
        pc_msg_t plane_msg;
        pc_msg_t planes_cloud;
        pcPtr plane_cloud(new pc_t);

        for (int i = 0; i < params.n_planes; i++)
        {
            this->seg.setInputCloud(remaining);
            this->seg.segment(*inliers, *coefficients);
            
            this->extract.setInputCloud(remaining);
            this->extract.setIndices(inliers);
            this->extract.setNegative(false);
            this->extract.filter(*plane_cloud);

            colorize(plane_cloud, (float)i / (float)params.n_planes);
            pcl::toROSMsg(*plane_cloud, plane_msg); 
            planes_sep.planes.push_back(plane_msg);
            planes_sep.header = planes_sep.planes[i].header;
            pcl::concatenatePointCloud(planes_cloud, plane_msg, planes_cloud);
            this->extract.setInputCloud(remaining);
            this->extract.setIndices(inliers);
            this->extract.setNegative(true);
            this->extract.filter(*remaining);
            planes.push_back(Eigen::Vector4f(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]));
            plane_eq.coef = {coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};
            coef_msg.planes.push_back(plane_eq);
            coef_msg.header = planes_sep.planes[i].header;
        }  
        this->publish(this->pub_planes_pc_col, planes_cloud, "world");    
        this->pub_planes.publish(coef_msg); 
        this->pub_planes_pc.publish(planes_sep); 
    }
    return planes;
}

void colorize(pcPtr input, float ratio)
{
    int r;
    int g;
    int b;

    int cycle = (int) (ratio * 6);
    int up = (int) ((6 * ratio - cycle) * 255);
    int down = 255 - up;

    switch(cycle)
    {
        case 0:
            r = 255;
            g = up;
            b = 0;
            break;
        case 1:
            r = down;
            g = 255;
            b = 0;
            break;
        case 2:
            r = 0;
            g = 255;
            b = up;
            break;
        case 3:
            r = 0;
            g = down;
            b = 255;
            break;
        case 4:
            r = up;
            g = 0;
            b = 255;
            break;
        case 5:
            r = 255;
            g = 0;
            b = down;
            break;
    }

    for (int i = 0; i < input->points.size(); i++)
    {
        input->points[i].r = r;
        input->points[i].g = g;
        input->points[i].b = b;
    }
}