// std
	#include <string>
	#include <iostream>
	#include <math.h>
    #include <Eigen/Core>
// ros
	#include <ros/console.h>
	#include <ros/callback_queue.h>
	#include <ros/callback_queue_interface.h>
// point cloud
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/impl/point_types.hpp>
	#include <pcl_ros/transforms.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/passthrough.h>
	#include <pcl/filters/radius_outlier_removal.h>
	#include <pcl/filters/statistical_outlier_removal.h>
	#include <pcl/keypoints/iss_3d.h>
	#include <pcl/keypoints/sift_keypoint.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl/kdtree/kdtree_flann.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/registration/correspondence_estimation.h>
    #include <pcl/registration/transformation_estimation_svd.h>
    #include <pcl/registration/correspondence_rejection_one_to_one.h>
    #include <pcl/registration/correspondence_rejection_sample_consensus.h>
    #include <pcl/features/vfh.h>
    #include <pcl/features/fpfh.h>
    #include <pcl/features/shot.h>
    #include <pcl/filters/extract_indices.h>
// tf
	#include <tf/LinearMath/Transform.h>
	#include <tf_conversions/tf_eigen.h>
	#include <tf/transform_listener.h>
	#include <tf/transform_broadcaster.h>
// synchronization
	#include <message_filters/subscriber.h>
	#include <message_filters/time_synchronizer.h>
	#include <message_filters/sync_policies/approximate_time.h>
// custom
	#include <geometry/Line.h>
	#include <geometry/Plane.h>
	#include <geometry/PointCloud.h>
	#include <registration/TransformEstimator.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
/*
*   args parser
*
*   node frequency
*   input topics namespace
*   output topics namespace
*/
float rad = 0.2;
vector <string> inputs;
string path; 
string output;
float frequency = 0;
ros::Publisher pub_kp1;
ros::Publisher pub_kp2;
tf::StampedTransform transf; 
const string sub_topic_name = "/reconstruction/planes/";


std::string get_topic_name(int input_number, int plane)
{
	return sub_topic_name + inputs[input_number] + "/plane" + patch::to_string(plane);
}

bool tf_exists(tf::TransformListener* tf_listener, string name)
{
    bool exists = false;
	ros::Duration(0.1).sleep();
	try
	{
        exists = tf_listener->waitForTransform("/cam_center", name, ros::Time::now(), ros::Duration(0.5));
	}
	catch (...)
	{
		ROS_DEBUG_STREAM(name << " not found");
	}
	return exists;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert_to_PCL(const PointCloud2ConstPtr& msgPtr)
{
    PointCloud2 msg = *msgPtr;
    pcl::PCLPointCloud2* pc2 = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr pc2Ptr(pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl_conversions::toPCL(msg, *pc2);
    pcl::fromPCLPointCloud2(*pc2Ptr, *pclPtr);

    return pclPtr;
}

pcl::PointCloud<pcl::Normal>::Ptr compute_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPtr)
{
    pcl::PointCloud<pcl::Normal>::Ptr normalsPtr (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    ne.setInputCloud(pclPtr);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.02);
    ne.compute(*normalsPtr);

    return normalsPtr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_nans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNans)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices); 
    pcl::removeNaNFromPointCloud(*cloudNans, indices->indices); 
    pcl::ExtractIndices<pcl::PointXYZRGB> rm_nan; 
    rm_nan.setInputCloud(cloudNans); 
    rm_nan.setIndices(indices); 
    rm_nan.setNegative(false); 
    rm_nan.filter(*cloud);

    return cloud; 
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_est(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPtr, pcl::PointCloud<pcl::Normal>::Ptr normalsPtr)
{
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308> ());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    vfh.setSearchMethod(tree);
    vfh.setInputCloud(pclPtr);
    vfh.setInputNormals(normalsPtr);
    vfh.setRadiusSearch(rad);
    vfh.compute(*vfhs);

    return vfhs;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_est(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPtr, pcl::PointCloud<pcl::Normal>::Ptr normalsPtr)
{
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    fpfh.setSearchMethod(tree);
    fpfh.setInputCloud(pclPtr);
    fpfh.setInputNormals(normalsPtr);
    fpfh.setRadiusSearch(rad);
    fpfh.compute(*fpfhs);

    return fpfhs;
}

pcl::PointCloud<pcl::SHOT352>::Ptr shot_est(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPtr, pcl::PointCloud<pcl::Normal>::Ptr normalsPtr)
{
    pcl::SHOTEstimation< pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352 > shot_e;
    pcl::PointCloud<pcl::SHOT352>::Ptr shot_f(new pcl::PointCloud<pcl::SHOT352>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    shot_e.setSearchMethod(tree); 
    shot_e.setInputCloud(pclPtr);
    shot_e.setInputNormals(normalsPtr); 
    shot_e.setRadiusSearch(rad);
    shot_e.compute(*shot_f);

    return shot_f;
}

pcl::CorrespondencesPtr compute_corresp(pcl::PointCloud<pcl::FPFHSignature33>::Ptr feat1, pcl::PointCloud<pcl::FPFHSignature33>::Ptr feat2)
{
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(feat1);
    est.setInputTarget(feat2);
    est.determineCorrespondences(*correspondences);

    return correspondences;
}

/////////// TODO: add planes !!!
Eigen::Matrix4f transform_est(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input2, pcl::CorrespondencesPtr corr, tf::TransformListener* listener)
{
    // pcl::registration::TransformationEstimationSVD <pcl::PointXYZRGB, pcl::PointXYZRGB> test;
    Eigen::Matrix4f transf;
    // test.estimateRigidTransformation(*input1, *input2, *corr, transf);
    TransformEstimator* TE = new TransformEstimator();
    vector<Eigen::Vector3f> vec3;
    for (int i = 0; i < input1->points.size(); i++)
    {
        vec3.push_back(Eigen::Vector3f(input1->points[i].x, input1->points[i].y, input1->points[i].z));
    }
    if (vec3.size() > 0)
    {
        TE->addPoints(vec3, true);
    }
    vec3.clear();
    for (int i = 0; i < input2->points.size(); i++)
    {
        vec3.push_back(Eigen::Vector3f((float) input2->points[i].x, (float) input2->points[i].y, (float) input2->points[i].z));
    }
    if (vec3.size() > 0)
    {
        TE->addPoints(vec3, false);
    }

    tf::StampedTransform plane_tf1;
    tf::StampedTransform plane_tf2;
    vector<geometry::Plane> planes_cam1;
    vector<geometry::Plane> planes_cam2;
    for (int i = 0; i < 2; i++)
    {
        // if (tf_exists(listener, get_topic_name(0, i+1)) && tf_exists(listener, get_topic_name(1, i+1)))
        // {
            listener->lookupTransform(get_topic_name(0, i+1), "cam_center", ros::Time::now(), plane_tf1);
            planes_cam1.push_back(geometry::Plane(plane_tf1));
            listener->lookupTransform(get_topic_name(1, i+1), "cam_center", ros::Time::now(), plane_tf2);
            planes_cam2.push_back(geometry::Plane(plane_tf2));
        // }
    }
    ROS_INFO_STREAM(planes_cam1.size() << " planes matches");
    if (planes_cam1.size() > 0)
    {
        TE->addPlanes(planes_cam1, true);
        TE->addPlanes(planes_cam2, false);
    }
    transf = TE->getTransform();

    return transf;
}


void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2, tf::TransformListener* listener)
{
    // conversions

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input2(new pcl::PointCloud<pcl::PointXYZRGB>);
        input1 = remove_nans(convert_to_PCL(pc1));
        input2 = remove_nans(convert_to_PCL(pc2));
    
    // normals
        pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
        normals1 = compute_normals(input1);
        normals2 = compute_normals(input2);

    // features
        // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33> ());
        // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33> ());
        // fpfhs1 = fpfh_est(input1, normals1);
        // fpfhs2 = fpfh_est(input2, normals2);

        // pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs1(new pcl::PointCloud<pcl::VFHSignature308> ());
        // pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs2(new pcl::PointCloud<pcl::VFHSignature308> ());
        // vfhs1 = fpfh_est(input1, normals1);
        // vfhs2 = fpfh_est(input2, normals2);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr feat1(new pcl::PointCloud<pcl::FPFHSignature33> ());
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr feat2(new pcl::PointCloud<pcl::FPFHSignature33> ());
        feat1 = fpfh_est(input1, normals1);
        feat2 = fpfh_est(input2, normals2);

        ROS_WARN_STREAM("n features 1: " << feat1->points.size());
        ROS_WARN_STREAM("n features 1: " << feat2->points.size());

    // matching  
        pcl::CorrespondencesPtr corr;
        corr = compute_corresp(feat1, feat2);

    // Duplication rejection Duplicate
        pcl::CorrespondencesPtr corr_dupl_rej(new pcl::Correspondences());
        pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
        corr_rej_one_to_one.setInputCorrespondences(corr);
        corr_rej_one_to_one.getCorrespondences(*corr_dupl_rej);


    // Correspondance rejection RANSAC
        // Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
        // pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector_sac;
        // pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
        // rejector_sac.setInputSource(input1);
        // rejector_sac.setInputTarget(input2);
        // rejector_sac.setInlierThreshold(0.02); // distance in m, not the squared distance
        // rejector_sac.setMaximumIterations(100000);
        // rejector_sac.setRefineModel(false);
        // rejector_sac.setInputCorrespondences(corr_dupl_rej);;
        // rejector_sac.getCorrespondences(*correspondences_filtered);
        // corr_dupl_rej.swap(correspondences_filtered);
        // transf = rejector_sac.getBestTransformation();   // Transformation Estimation method 1

    Eigen::Matrix4f H;
    pcl::Correspondences fin_corr(*corr_dupl_rej);
    ROS_INFO_STREAM(fin_corr.size() << " correspondances");
    for (int i = 0; i < fin_corr.size(); i++)
    {
        ROS_INFO_STREAM(fin_corr[i].index_query << " -> " << fin_corr[i].index_match << " (" << fin_corr[i].distance << ")");
    }


    H = transform_est(input1, input2, corr_dupl_rej, listener);

    // // listener->lookupTransform("registration_" + inputs[0] + "_" + inputs[1], name, ros::Time(0), *transf);

    tf::Transform tf;
    tf::Vector3 origin;
    tf::Matrix3x3 tf3d;
    tf::Quaternion tfqt;

    
    origin.setValue(static_cast<double>(H(0,3)),static_cast<double>(H(1,3)),static_cast<double>(H(2,3)));
    tf3d.setValue(static_cast<double>(H(0,0)), static_cast<double>(H(0,1)), static_cast<double>(H(0,2)), 
                    static_cast<double>(H(1,0)), static_cast<double>(H(1,1)), static_cast<double>(H(1,2)), 
                    static_cast<double>(H(2,0)), static_cast<double>(H(2,1)), static_cast<double>(H(2,2)));
    tf3d.getRotation(tfqt);
    tf.setOrigin(origin);
    tf.setRotation(tfqt);



    pcl::PointCloud<pcl::PointXYZ>::Ptr kp1 (new pcl::PointCloud<pcl::PointXYZ>);
    kp1->width = fin_corr.size();
    kp1->height = 1;
    kp1->resize(kp1->width);
    for (int i = 0; i < fin_corr.size(); i++)
    {
        kp1->points[i].x = input1->points[fin_corr[i].index_query].x;
        kp1->points[i].y = input1->points[fin_corr[i].index_query].y;
        kp1->points[i].z = input1->points[fin_corr[i].index_query].z;
    }
    pcl::PCLPointCloud2 msg1;
    pcl::toPCLPointCloud2(*kp1, msg1);
    sensor_msgs::PointCloud2 out1;
    pcl_conversions::fromPCL(msg1, out1);
    out1.header.frame_id = "cam_center";

    pcl::PointCloud<pcl::PointXYZ>::Ptr kp2 (new pcl::PointCloud<pcl::PointXYZ>);
    kp2->width = fin_corr.size();
    kp2->height = 1;
    kp2->resize(kp1->width);
    for (int i = 0; i < fin_corr.size(); i++)
    {
        kp2->points[i].x = input2->points[fin_corr[i].index_match].x;
        kp2->points[i].y = input2->points[fin_corr[i].index_match].y;
        kp2->points[i].z = input2->points[fin_corr[i].index_match].z;
    }
    pcl::PCLPointCloud2 msg2;
    pcl::toPCLPointCloud2(*kp2, msg2);
    sensor_msgs::PointCloud2 out2;
    pcl_conversions::fromPCL(msg2, out2);
    out2.header.frame_id = "cam_center";

    // PointCloud2 merged;
    // try
    // {    
    //     pcl_ros::transformPointCloud("cam_center", tf, out1, out1);
    // }
    // catch (tf::TransformException ex)
    // {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }
     
    // pcl::concatenatePointCloud(out1, out2, merged);

    static tf::TransformBroadcaster br; 
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "cam_center", "/" + inputs[0] + "_" + inputs[1]));
    // pub_pc.publish(merged);

    pub_kp1.publish(out1);
    pub_kp2.publish(out2);
}

int main(int argc, char *argv[])
{

    // parsing arguments

		inputs.clear();

        
        inputs.push_back(argv[1]);
        inputs.push_back(argv[2]);
        path = argv[3];
        output = argv[4];
		string fr(argv[5]);
		frequency = (float)atof(fr.c_str());

    // Initialize ROS
        string node_name = "correspondance_" + inputs[0] + "_" + inputs[1];
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;
        tf::TransformListener* tf_listener;

    // Synchronize both kinects messages
        message_filters::Subscriber<PointCloud2> cam1(nh, path + "/" + inputs[0] + "/keypoints", 1);
        message_filters::Subscriber<PointCloud2> cam2(nh, path + "/" + inputs[1] + "/keypoints", 1);
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2, tf_listener));

        pub_kp1 = nh.advertise<sensor_msgs::PointCloud2>(path + "/" + inputs[0] + "/keypoints_matches", 1);
        pub_kp2 = nh.advertise<sensor_msgs::PointCloud2>(path + "/" + inputs[1] + "/keypoints_matches", 1);

    // ROS loop
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        
        // sleep
            if (frequency > 0)
            {
                loop_rate.sleep();
                ros::spinOnce();
            }
            else
            {
                ros::spin();
            }
    }

    return 0;
}