#include "PlaneDetector.h"

PlaneDetector::PlaneDetector(ros::NodeHandle* node, string name, string topic_in)
{
    this->name = name;
    string topic_out = "/calib/planes/" + this->name;
    this->node = node;
	this->tf_listener = new tf::TransformListener; 
    this->sub = this->node->subscribe(topic_in, 1, &PlaneDetector::update, this);
    this->pub_planes_col = this->node->advertise<pc_msg_t>(topic_out + "/color", 1);
    this->pub_planes = this->node->advertise<calib::Planes>(topic_out + "/planes", 1);

    this->seg.setOptimizeCoefficients(true);
    this->seg.setModelType(pcl::SACMODEL_PLANE);
}
 
void PlaneDetector::conf_callback(calib::PlaneConfig &config, uint32_t level)
{
    this->param_plane.method = config.method;
    this->param_plane.n_planes = config.n_planes;
    this->param_plane.th_dist = config.th_dist / 1000;
    this->param_plane.max_it = config.max_it;
}

void PlaneDetector::update(const pcConstPtr& input)
{
    pc_t* cloud(new pc_t(*input));
    pcPtr cloudPtr(cloud); 
    this->detect_plane(cloudPtr, this->param_plane);
}

/*
* @brief Detect planes in a point cloud and publish a segmented point cloud.
*
* @param input Input point cloud.
* @param params Plane detection parameters.
*/
void PlaneDetector::detect_plane(const pcPtr& input, param_plane_t params)
{
    if (params.method >= 0 && params.n_planes && params.th_dist >= 0 && params.max_it > 0 && input->points.size() > 0)
    {
        seg.setMethodType(params.method);
        this->seg.setMaxIterations(params.max_it);
        this->seg.setDistanceThreshold (params.th_dist);
        pcPtr remaining(new pc_t);
        remaining = input;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        calib::Planes msg;
        shape_msgs::Plane plane_eq;
        pc_msg_t color_cloud;
        pc_msg_t temp_msg;
        pcPtr temp_cloud(new pc_t);
        pcl::toROSMsg(*input, temp_msg); 
        msg.point_cloud = temp_msg;
        for (int i = 0; i < params.n_planes; i++)
        {
            this->seg.setInputCloud(remaining);
            this->seg.segment(*inliers, *coefficients);
            
            this->extract.setInputCloud(remaining);
            this->extract.setIndices(inliers);
            this->extract.setNegative(false);
            this->extract.filter(*temp_cloud);

            colorize(temp_cloud, (float)i / (float)params.n_planes);
            pcl::toROSMsg(*temp_cloud, temp_msg); 
            msg.clouds.push_back(temp_msg);
            msg.header = temp_msg.header;
            pcl::concatenatePointCloud(color_cloud, temp_msg, color_cloud);
            this->extract.setInputCloud(remaining);
            this->extract.setIndices(inliers);
            this->extract.setNegative(true);
            this->extract.filter(*remaining);
            plane_eq.coef = {coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]};
            msg.planes.push_back(plane_eq);
        }  
    ROS_DEBUG_STREAM("Publishing " << msg.clouds.size() << " planes.");
    ROS_DEBUG_STREAM("Publishing colored point cloud (" << color_cloud.data.size() << " points).");   
    this->pub_planes_col.publish(color_cloud); 
    this->pub_planes.publish(msg); 
    }
}