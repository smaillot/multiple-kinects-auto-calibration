#ifndef DEF_MERGING
#define DEF_MERGING

#include <string>
#include <ros/console.h>

#include <shape_msgs/Plane.h>
#include <calib/Cloud.h>
#include <calib/Planes.h>
#include <calib/PlaneClouds.h>
#include <calib/MergingConfig.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/keypoints/iss_3d.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

pcPtr colorize(vector<pcPtr> input, pcl::CorrespondencesPtr matches, bool match);
pcPtr colorize(calib::PlaneClouds input1, calib::PlaneClouds input2, pcl::CorrespondencesPtr matches);
pcPtr colorize(pcPtr input1, pcPtr input2, pcl::CorrespondencesPtr matches);

class Merging
{
    private:
        std::string name1;
        std::string name2;
        ros::NodeHandle* node;
        ros::Subscriber sub_pc1;
        ros::Subscriber sub_pc2;
        ros::Subscriber sub_full_pc1;
        ros::Subscriber sub_full_pc2;
        ros::Subscriber sub_coef1;
        ros::Subscriber sub_coef2;
        bool callback[6];
        calib::PlaneClouds pc1;
        calib::PlaneClouds pc2; 
        pcPtr full_pc1;
        pcPtr full_pc2;     
        pcPtr keypoints1;
        pcPtr keypoints2;      
        calib::Planes coef1;
        calib::Planes coef2;
	    ros::Publisher pub_color;
	    ros::Publisher pub_kp;

        float radius;
        bool kp_dupl_rej;
        float kp_est_radius;
        float iss_support_radius;
        float iss_nms_radius;
	    param_cut_t param_cut;
	    float match_th;

        pcl::search::KdTree<Point>::Ptr kdtree;
        pcl::NormalEstimation<Point, pcl::Normal> norm;
        pcl::VFHEstimation<Point, pcl::Normal, global_desc_t> vfh;
        /**/pcl::FPFHEstimation<Point, pcl::Normal, kp_t> kp_feat_est;/**/
        /*/pcl::SHOTEstimation<Point, pcl::Normal, kp_t> kp_feat_est;/**/
        pcl::registration::CorrespondenceEstimation<global_desc_t, global_desc_t> corr_est;
        pcl::registration::CorrespondenceRejectorOneToOne corr_rej;
		pcl::ISSKeypoint3D<Point, Point> iss;
        pcl::PassThrough<Point> filter_cut;
        

    public:
        Merging(ros::NodeHandle* node, std::string pc1, std::string pc2);

        void pc1_callback(const calib::PlaneCloudsConstPtr& input);
        void pc2_callback(const calib::PlaneCloudsConstPtr& input);
        void coef1_callback(const calib::PlanesConstPtr& input);
        void coef2_callback(const calib::PlanesConstPtr& input);
        void full_pc1_callback(const pcConstPtr& input);
        void full_pc2_callback(const pcConstPtr& input);
        bool isSync();        
        void callback_sync();
        void conf_callback(calib::MergingConfig &config, uint32_t level);

        pc_nPtr normals(pcPtr input, float radius);
        pc_featPtr descriptor(pcPtr input, pc_nPtr normals);
        pcl::CorrespondencesPtr match_planes(pc_featPtr desc1, pc_featPtr desc2);
        pcl::CorrespondencesPtr get_corr();
        kp_featPtr feat_est(pcPtr cloudPtr, pc_nPtr normalsPtr);
        pcl::CorrespondencesPtr compute_kp_corr(kp_featPtr feat1, kp_featPtr feat2);
        pcl::CorrespondencesPtr get_kp_corr();
        pcPtr extract_kp(pcPtr cloudPtr);
        pcPtr cut(pcPtr input, param_cut_t params);
        pcPtr remove_nans(pcPtr cloudNans);
};

#endif