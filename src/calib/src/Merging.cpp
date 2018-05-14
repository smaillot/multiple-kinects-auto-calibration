#include "Merging.h"

using namespace std;
using namespace message_filters;

/*
* @brief Constructor
*/
Merging::Merging(ros::NodeHandle node, string pc1, string pc2)
{
    this->pc1 = pc1;
    this->pc2 = pc2;
    this->node = node;

	message_filters::Subscriber<calib::PlaneClouds> sub1(this->node, "/calib/clouds/" + pc1 + "/planes", 1);
	message_filters::Subscriber<calib::PlaneClouds> sub2(this->node, "/calib/clouds/" + pc2 + "/planes", 1);
	typedef sync_policies::ApproximateTime<calib::PlaneClouds, calib::PlaneClouds> PlanesSync;
	Synchronizer<PlanesSync> sync(PlanesSync(10), sub1, sub2);
	sync.registerCallback(boost::bind(&Merging::planes_callback, this, _1, _2));

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    this->kdtree = tree;
}

void Merging::planes_callback(calib::PlaneClouds planes1, calib::PlaneClouds planes2)
{

}

/*
* @brief Compute point cloud normals.
*
* @param input Point cloud.
* @param radius Search radius for normal estimation.
*/
pcl::PointCloud<pcl::Normal>::Ptr Merging::normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, float radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    this->norm.setInputCloud(input);
    this->norm.setRadiusSearch(radius);
    this->norm.setSearchMethod(this->kdtree);
    this->norm.compute(*normals);
    return normals;
}

/*
* @brief Compute VFH global descritpor from a point cloud.
*
* @param input Point cloud.
* @param normals Point cloud estimated normals.
*/
pcl::PointCloud<pcl::VFHSignature308>::Ptr Merging::descriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
    this->vfh.setInputCloud(input);
	this->vfh.setInputNormals(normals);
	this->vfh.setSearchMethod(this->kdtree);
    this->vfh.setNormalizeBins(true);
    this->vfh.setNormalizeDistance(false);
    this->vfh.compute(*descriptor);
    return descriptor;
}

/*
* @brief Match detected planes 
*
* @param desc1 Point cloud containing VFH descriptor for each plane of the first cloud.
* @param desc2 Point cloud containing VFH descriptor for each plane of the second cloud.
*/
pcl::CorrespondencesPtr Merging::match_planes(pcl::PointCloud<pcl::VFHSignature308>::Ptr desc1, pcl::PointCloud<pcl::VFHSignature308>::Ptr desc2)
{
    pcl::CorrespondencesPtr corr(new pcl::Correspondences());
    this->corr_est.setInputSource(desc1);
    this->corr_est.setInputTarget(desc2);
    this->corr_est.determineCorrespondences(*corr);

    // this->corr_rej.setInputCorrespondences(corr);
    // this->corr_rej.getCorrespondences(*corr);

    ROS_DEBUG_STREAM("Correspondances:");
    for (int i = 0; i < corr->size(); i++)
    {
        ROS_DEBUG_STREAM("Plane" << (*corr)[i].index_query << " -> Plane" << (*corr)[i].index_match << " (" << (*corr)[i].distance << ")");
    }

    return corr;
}