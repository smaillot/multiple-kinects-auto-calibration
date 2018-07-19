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

Cloud::Cloud()
{}

Cloud::Cloud(ros::NodeHandle* node, string sub_name, string pub_name, bool icp)
{
    this->name = pub_name;
    this->icp = icp;
    string topic = "/calib/clouds/" + pub_name;
    ROS_DEBUG_STREAM("Init Cloud instance " << pub_name << ": " << sub_name);
    ROS_DEBUG_STREAM("\tpublish topic: " << topic);
    this->node = node;
	this->tf_listener = new tf::TransformListener; 
    this->frame = "world";
    this->frame_pub = "cam_center";

    this->sub = this->node->subscribe(sub_name, 1, &Cloud::update, this);
    this->pub_raw = this->node->advertise<pc_t>(topic, 1);
}

void Cloud::conf_callback(calib::CloudConfig &config, uint32_t level)
{
    ROS_DEBUG_STREAM_ONCE("Init " << this->name << " Cloud parameters");

    // transform
    this->param_transform.tx = config.tx / 1000;
    this->param_transform.ty = config.ty / 1000;
    this->param_transform.tz = config.tz / 1000;
    this->param_transform.rx = config.rx / 180 * 3.14159;
    this->param_transform.ry = config.ry / 180 * 3.14159;
    this->param_transform.rz = config.rz / 180 * 3.14159;
}

// publishers
    void Cloud::publish(ros::Publisher& pub, pc_t& cloud)
    {
        pc_msg_t msg;
        pcl::toROSMsg(cloud, msg);
        this->publish(pub, msg);
    }

    void Cloud::publish(ros::Publisher& pub, const pcConstPtr& cloudPtr)
    {
        pc_t cloud = *cloudPtr; 
        this->publish(pub, cloud);
    }

    void Cloud::publish(ros::Publisher& pub, const pc_msg_t& input)
    {
        pc_msg_t msg = input;
        pcl_ros::transformPointCloud(this->frame_pub, msg, msg, *this->tf_listener);  

        if (msg.data.size() > 0)
        {
            pub.publish(msg);
        }
    }

void Cloud::update(const pcConstPtr& input)
{
    ROS_DEBUG_STREAM("Receiving " << input->points.size() << " points for " << this->name << "(" << this->frame << ").");
    pc_t* cloud(new pc_t(*input));
    pcPtr cloudPtr(cloud); 

    if (this->icp)
    {
        tf::StampedTransform icp_tf;
        tf_listener->waitForTransform("world", this->name + "_icp", ros::Time(0), ros::Duration(5.0));
        pcl_ros::transformPointCloud("world", *cloud, *cloud, *this->tf_listener); 
        tf_listener->lookupTransform("world", this->name + "_icp", ros::Time(0), icp_tf);
        pcl_ros::transformPointCloud(*cloud, *cloud, icp_tf);
    }

    pcl_ros::transformPointCloud(this->frame, *cloud, *cloud, *this->tf_listener);  
    pcl_ros::transformPointCloud(*cloud, *cloud, this->get_transform(this->param_transform, true, true));  
    if (this->pub_raw.getTopic() != this->sub.getTopic())     
    {
        ROS_DEBUG_STREAM("Publishing " << cloudPtr->points.size() << " for " << this->name << " raw point cloud " << "(" << this->frame_pub << ").");
        this->publish(this->pub_raw, cloudPtr);
    }
    this->cloud = cloudPtr;
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

pcPtr Cloud::remove_nans(pcPtr cloudNans)
{
    pcPtr cloud(new pc_t);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices); 
    pcl::removeNaNFromPointCloud(*cloudNans, indices->indices); 
    pcl::ExtractIndices<Point> rm_nan; 
    rm_nan.setInputCloud(cloudNans); 
    rm_nan.setIndices(indices); 
    rm_nan.setNegative(false); 
    rm_nan.filter(*cloud);

    return cloud; 
}