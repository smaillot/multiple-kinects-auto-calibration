#include "plane_detection/PlaneDetector.h"

using namespace geometry;

/**
 * @brief Constructor.
 *
 * @params n_planes Number of planes to detect
 * @params PC Pointer on a custom PointCloud object from geometry package
 */
 PlaneDetector::PlaneDetector(ros::NodeHandle nh, std::string topic_name, std::string pub_name)
    : seg()
{ 
    this->node = nh;
    this->sub_name = topic_name;
    this->pub_name = pub_name;
    this->tf_listener = new tf::TransformListener; 

    // init segmentation model

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);

    // subscriber 

        this->pc_sub = this->node.subscribe(this->sub_name, 1, &PlaneDetector::update, this);
}

/**
 * @brief Setter for plane detection parameters.
 * 
 * @param enabled Plane detection activation.
 * @param n_planes Number of planes to detect.
 * @param th_dist Maximum distance to be considered in the support of the plane (RANSAC).
 * @param max_it Maximum number of iterations.
 */
void PlaneDetector::set_params(bool enabled, int n_planes, float th_dist, int max_it)
{
    this->enabled = enabled;
    this->n_planes = n_planes;
    this->th_dist = th_dist;
    this->max_it = max_it;
}

/**
 * @brief Update point cloud from topic.
 * 
 * @param cloud Input cloud message.
 */
void PlaneDetector::update(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    this->detect_planes();
}

/**
 * @brief Default constructor.
 */
void PlaneDetector::detect_planes()
{
    // init variables
        pcl::PCLPointCloud2* input_cloud = new pcl::PCLPointCloud2(*this->cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*input_cloud, *temp_cloud);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // update segmentation model
        this->seg.setDistanceThreshold (this->th_dist);
        this->seg.setMaxIterations(this->max_it);

    for (int i = 0 ; i < this->n_planes ; i++)
    {
        // apply filter
            seg.setInputCloud(temp_cloud);
            seg.segment(*inliers, *coefficients);

        // geometry computation

            // translation
                tf::Vector3 normal_vect(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
                Eigen::Matrix< double, 4, 1 > transl;
                pcl::compute3DCentroid(*temp_cloud, *inliers, transl);
                tf::Vector3 translation(transl(0, 0), transl(1, 0), transl(2, 0));
                
            // rotation
                tf::Vector3 z(0, 0, 1);
                double angle = z.angle(normal_vect);
                tf::Vector3 axis = z.cross(normal_vect);
                tf::Quaternion Q;
                Q.setRotation(axis, angle);

            // convert into tf
                tf::Transform transform;
                transform.setOrigin(translation);
                transform.setRotation(Q);
                
            // brodcast
                static tf::TransformBroadcaster br; 
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/cam_center", pub_name + patch::to_string(this->n_planes))); 
            
            //subtract inliers
                pcl::ExtractIndices<pcl::PointXYZ> extract_neg;
                extract_neg.setInputCloud(temp_cloud);
                extract_neg.setIndices(inliers);
                extract_neg.setNegative(true);
                extract_neg.filter(*temp_cloud);
    }
}

