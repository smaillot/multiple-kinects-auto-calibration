#ifndef GEOMETRY_PLANE_DETECTOR
#define GEOMETRY_PLANE_DETECTOR

// std
    #include <string>
    #include <vector>
    #include <iostream>
// ros
    #include <ros/console.h>
// pcl
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/segmentation/sac_segmentation.h>
	#include <pcl/filters/extract_indices.h>
	#include <pcl_conversions/pcl_conversions.h>
// My libraries
    #include <geometry/Line.h>
    #include <geometry/Plane.h>
    #include <geometry/PointCloud.h>


namespace geometry
{
    class PlaneDetector
    {
        private:
            // ros node interaction
                ros::NodeHandle node;  
                const tf::TransformListener* tf_listener;
                std::string reference_frame;

                std::string sub_name;  
                std::string pub_name;
                ros::Subscriber pc_sub;
                std::vector <ros::Publisher> planes_pub;
        
            // parameters

                bool enabled;
                int n_planes;
                float th_dist;
                int max_it;

            // Point cloud
                pcl::PCLPointCloud2* cloud;
                pcl::SACSegmentation<pcl::PointXYZRGB> seg;

            // Results
                std::vector<Plane> planes;

        public:

            PlaneDetector(ros::NodeHandle nh, std::string topic_name, std::string pub_name, std::string frame);

            void set_params(bool enabled, int n_planes, float th_dist, int max_it);

            void update(const sensor_msgs::PointCloud2ConstPtr& cloud);
            void detect_planes();
            

    };
};

#endif