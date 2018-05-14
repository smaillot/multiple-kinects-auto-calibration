#ifndef DEF_MERGING
#define DEF_MERGING

#include <string>
#include <ros/console.h>

#include <shape_msgs/Plane.h>
#include <calib/Planes.h>
#include <calib/PlaneClouds.h>

#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Merging
{
    private:
        std::string pc1;
        std::string pc2;
        ros::NodeHandle node;

    	message_filters::Subscriber<calib::PlaneClouds> sub1;
    	message_filters::Subscriber<calib::PlaneClouds> sub2;

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm;
        pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
        pcl::registration::CorrespondenceEstimation<pcl::VFHSignature308, pcl::VFHSignature308> corr_est;
        pcl::registration::CorrespondenceRejectorOneToOne corr_rej;

    public:
        Merging(ros::NodeHandle node, std::string pc1, std::string pc2);

        void planes_callback(calib::PlaneClouds planes1, calib::PlaneClouds planes2);

        pcl::PointCloud<pcl::Normal>::Ptr normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float radius);
        pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::Normal>::Ptr normals);
        pcl::CorrespondencesPtr match_planes(pcl::PointCloud<pcl::VFHSignature308>::Ptr desc1, pcl::PointCloud<pcl::VFHSignature308>::Ptr desc2);
};

#endif