#ifndef DEF_MATCHING
#define DEF_MATCHING

#include <string>
#include <ros/console.h>
#include <algorithm> 

#include "Cloud.h"
#include "Preprocessing.h"

#include <shape_msgs/Plane.h>
#include <calib/Cloud.h>
#include <calib/Planes.h>
#include <calib/Matches.h>
#include <calib/MatchingConfig.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>

#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/keypoints/iss_3d.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointCloud<pcl::Normal> pc_n_t;
typedef pc_n_t::Ptr pc_nPtr;
typedef pcl::VFHSignature308 global_desc_t;
typedef pcl::PointCloud<global_desc_t> plane_feat_t;
typedef plane_feat_t::Ptr pc_featPtr;

/*/typedef pcl::FPFHSignature33 kp_t;
    typedef pcl::FPFHEstimation<Point, pcl::Normal, kp_t> kp_est_t; /**/
/*/ typedef pcl::VFHSignature308 kp_t;
    typedef pcl::VFHEstimation<Point, pcl::Normal, kp_t> kp_est_t; /**/
/**/ typedef pcl::SHOT1344 kp_t; //352
typedef pcl::SHOTColorEstimationOMP<Point, pcl::Normal, kp_t> kp_est_t; /**/

typedef pcl::PointCloud<kp_t> kp_feat_t;
typedef kp_feat_t::Ptr kp_featPtr;

void colorize(vector<pcPtr> input, pcl::CorrespondencesPtr matches, bool match, pcPtr output);
void colorize(calib::Planes input1, calib::Planes input2, pcl::CorrespondencesPtr matches, pcPtr output);
void colorize(pcPtr& input1, pcPtr& input2, pcl::CorrespondencesPtr& matches, pcPtr& output);

void convert(pc_msg_t input, pc_msgPtr output);
void convert(const pc_msgPtr& input, pcl::PCLPointCloud2Ptr& output);
void convert(const pcl::PCLPointCloud2Ptr& input, pcPtr& output);
void convert(const pc_msgPtr& input, pcPtr& output);
void convert(pc_msg_t input, pcPtr output);
void convert(pc_msg_t input, pc_msgPtr output);

void publish(ros::Publisher& pub, pc_t& cloud);
void publish(ros::Publisher& pub, const pc_msg_t& input);

class Matching
{
    private:
        std::string name1;
        std::string name2;
        ros::NodeHandle* node;
        pcPtr keypoints1;
        pcPtr keypoints2; 
        pcPtr cloud1;
        pcPtr cloud2;      
        calib::Planes planes1;
        calib::Planes planes2;
	    ros::Publisher pub_color;
	    ros::Publisher pub_kp;
	    ros::Publisher pub_all_kp;
	    ros::Publisher pub_matches;
	    ros::Publisher pub_objects1;
	    ros::Publisher pub_objects2;
        string frame;

        float radius;
        bool kp_dupl_rej;
        float kp_est_radius;
        float iss_support_radius;
        float iss_nms_radius;
        float cut_reverse;
	    float cut_th;
	    float match_th;
        float match_th_dist;

        pcl::search::KdTree<Point>::Ptr kdtree;
        pcl::KdTreeFLANN<kp_t> match_search;
        pcl::NormalEstimation<Point, pcl::Normal> norm;
        pcl::VFHEstimation<Point, pcl::Normal, global_desc_t> global_est;
        /**/kp_est_t kp_feat_est;/**/
        pcl::registration::CorrespondenceEstimation<global_desc_t, global_desc_t> corr_est;
        pcl::registration::CorrespondenceRejectorSampleConsensus<Point> corr_rej;
		pcl::ISSKeypoint3D<Point, Point> iss;
        pcl::PassThrough<Point> filter_cut;
        

    public:
        Matching(ros::NodeHandle* node, std::string name1, std::string name2);

        void update(const calib::PlanesConstPtr& planes1, const calib::PlanesConstPtr& planes2);
        void callback_sync();
        void conf_callback(calib::MatchingConfig &config, uint32_t level);

        pc_nPtr normals(pcPtr input, float radius);
        pc_featPtr descriptor(pcPtr input, pc_nPtr normals);
        pcl::CorrespondencesPtr match_planes(pc_featPtr desc1, pc_featPtr desc2);
        pcl::CorrespondencesPtr get_corr();
        kp_featPtr feat_est(pcPtr& cloudPtr, pc_nPtr& normalsPtr, pcPtr& surfacePtr);
        pcl::CorrespondencesPtr compute_kp_corr(kp_featPtr feat1, kp_featPtr feat2);
        pcl::CorrespondencesPtr get_kp_corr();
        pcPtr extract_kp(pcPtr cloudPtr, calib::Planes planes, ros::Publisher pub);
        pcPtr cut(pcPtr input, param_cut_t params);
        pcPtr cut_plane(pcPtr input, calib::Planes plane);
        pcPtr remove_nans(pcPtr cloudNans);
};

#endif