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
    this->pub_planes = this->node->advertise<sensor_msgs::PointCloud2>(pub_name + "/planes", 1);

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

    /*
    * @brief Convert point cloud between pcl and sensor_msg objects.
    *
    * @param input Input point cloud.
    * @param output Output point cloud.
    */
    void Cloud::convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, sensor_msgs::PointCloud2& output)
    {
        pcl::PCLPointCloud2Ptr temp;
        sensor_msgs::PointCloud2ConstPtr temp2;
        this->convert(input, temp);
        this->convert(temp, temp2);
        output = *temp2;
    }

// publishers
    /*
    * @brief Publish point cloud.
    *
    * @param pub Ros message publisher.
    * @param msg Point cloud message to publish.
    */
    void Cloud::publish(ros::Publisher& pub, sensor_msgs::PointCloud2& msg, string frame)
    {
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
    void Cloud::publish(ros::Publisher& pub, const sensor_msgs::PointCloud2ConstPtr& msgPtr, string frame)
    {
        sensor_msgs::PointCloud2 msg = *msgPtr; 
        this->publish(pub, msg, frame);
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


Eigen::Matrix4f Cloud::get_transform(param_transform_t params, bool rot, bool tr)
{
    Transform<float, 3, Eigen::Affine> t;
    if (tr)
    {
        t = Translation<float, 3>(params.tx, params.ty, params.tz);
    }
    else
    {
        t = Translation<float, 3>(0, 0, 0);
    }
    if (rot)
    {
        t.rotate(AngleAxis<float>(params.rx, Vector3f::UnitX()));
        t.rotate(AngleAxis<float>(params.ry, Vector3f::UnitY()));
        t.rotate(AngleAxis<float>(params.rz, Vector3f::UnitZ()));
    }
    return t.matrix();
}

/*
* @brief Subscriber callback containing processing loop
*/
void Cloud::update(const sensor_msgs::PointCloud2ConstPtr& input)
{
    sensor_msgs::PointCloud2* msg(new sensor_msgs::PointCloud2(*input)); 
    pcl_ros::transformPointCloud("world", *msg, *msg, *this->tf_listener);  
    pcl_ros::transformPointCloud(this->get_transform(this->param_transform, true, true), *msg, *msg);  
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
    this->publish(this->pub_preproc, cloud2Ptr, "world");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>); 
    this->convert(cloud2Ptr, cloudPtr);
    this->planes = this->detect_plane(cloudPtr, this->param_plane);

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

/*
* @brief Detect planes in a point cloud and publish a segmented point cloud.
*
* @param input Input point cloud.
* @param params Plane detection parameters.
*/
vector <Eigen::Vector4f> Cloud::detect_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, param_plane_t params)
{
    vector <Eigen::Vector4f> planes;
    if (params.method >= 0 && params.n_planes && params.th_dist >= 0 && params.max_it > 0)
    {
        seg.setMethodType(params.method);
        this->seg.setDistanceThreshold (params.th_dist);
        this->seg.setMaxIterations(params.max_it);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZRGB>);
        remaining = input;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        sensor_msgs::PointCloud2 planes_msg;
        sensor_msgs::PointCloud2 plane_msg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < params.n_planes; i++)
        {
            this->seg.setInputCloud(remaining);
            this->seg.segment(*inliers, *coefficients);
            
            this->extract.setInputCloud(remaining);
            this->extract.setIndices(inliers);
            this->extract.setNegative(false);
            this->extract.filter(*plane_cloud);

            this->colorize(*plane_cloud, (float)i / (float)params.n_planes);
            this->convert(plane_cloud, plane_msg);
            pcl::concatenatePointCloud(planes_msg, plane_msg, planes_msg);

            this->extract.setInputCloud(remaining);
            this->extract.setIndices(inliers);
            this->extract.setNegative(true);
            this->extract.filter(*remaining);

            planes.push_back(Eigen::Vector4f(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]));
        }  
        this->publish(this->pub_planes, planes_msg, "world");    
    }
    return planes;
}

void Cloud::colorize(pcl::PointCloud<pcl::PointXYZRGB> input, float ratio)
{
    int r;
    int g;
    int b;

    int cycle = (int) (ratio * 6);
    int up = (int) (6 * ratio - cycle) * 255;
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

    for (int i = 0; i < input.points.size(); i++)
    {
        input.points[i].r = r;
        input.points[i].g = g;
        input.points[i].b = b;
    }
}