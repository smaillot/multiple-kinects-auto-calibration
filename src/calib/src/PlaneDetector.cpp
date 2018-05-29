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
    this->frame = config.frame;
    this->param_plane.method = config.method;
    this->param_plane.n_planes = config.n_planes;
    this->param_plane.th_dist = config.th_dist / 1000;
    this->param_plane.max_it = config.max_it;
    this->subsize = config.subsize / 1000;
}

void PlaneDetector::update(const pcConstPtr& input)
{
    pc_t* cloud(new pc_t(*input));
    pcPtr cloudPtr(cloud); 
    pcl_ros::transformPointCloud(this->frame, *cloud, *cloud, *this->tf_listener); 
    this->detect_plane(cloudPtr, this->param_plane);
}

/*
 * @brief Apply a voxel filter to the point cloud. 
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr PlaneDetector::subsample(const pcPtr& input, param_voxel_t params)
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
        remaining = this->subsample(input, {true, this->subsize, this->subsize, this->subsize});
        ROS_DEBUG_STREAM(remaining->points.size() << " points remaining after downsampling");
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
    
    string pub_frame;
    ros::param::param<string>("/calib/" + this->name + "/cloud/frame_pub", pub_frame, "world");
    pcl_ros::transformPointCloud(pub_frame, color_cloud, color_cloud, *this->tf_listener); 
    this->pub_planes_col.publish(color_cloud); 
    this->pub_planes.publish(msg); 
    }
}