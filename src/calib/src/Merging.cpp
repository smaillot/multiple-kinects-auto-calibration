#include "Merging.h"

using namespace std;
using namespace message_filters;

/*
* @brief Constructor
*/
Merging::Merging(ros::NodeHandle* node, string name1, string name2)
{
    this->name1 = name1;
    this->name2 = name2;
    this->node = node;
    for (int i = 0; i < 4; i++) this->callback[i] = false;

    this->sub_pc1 = this->node->subscribe("/calib/clouds/" + name1 + "/planes", 1, &Merging::pc1_callback, this);
    this->sub_pc2 = this->node->subscribe("/calib/clouds/" + name2 + "/planes", 1, &Merging::pc2_callback, this);
    this->sub_coef1 = this->node->subscribe("/calib/planes/" + name1, 1, &Merging::coef1_callback, this);
    this->sub_coef2 = this->node->subscribe("/calib/planes/" + name2, 1, &Merging::coef2_callback, this);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    this->kdtree = tree;
}

void Merging::pc1_callback(const calib::PlaneCloudsConstPtr& input)
{
    this->callback[0] = true;
    this->pc1 = *input;
    if (this->isSync())
    {
        // this->callback_sync();
    }
}
void Merging::pc2_callback(const calib::PlaneCloudsConstPtr& input)
{
    this->callback[1] = true;
    this->pc2 = *input;
    if (this->isSync())
    {
        // this->callback_sync();
    }
}
void Merging::coef1_callback(const calib::PlanesConstPtr& input)
{
    this->callback[2] = true;
    this->coef1 = *input;
    if (this->isSync())
    {
        // this->callback_sync();
    }
}
void Merging::coef2_callback(const calib::PlanesConstPtr& input)
{
    this->callback[3] = true;
    this->coef2 = *input;
    if (this->isSync())
    {
        // this->callback_sync();
    }
}

bool Merging::isSync()
{
    if (this->callback[0] == true && this->callback[1] == true && this->callback[2] == true && this->callback[3] == true)
    {
        for (int i = 0; i < 4; i++) this->callback[i] = false;
        return true;
    }
    else
    {
        return false;
    }
}

void Merging::callback_sync()
{
    pcl::PointCloud<pcl::VFHSignature308> desc1;
    pcl::PointCloud<pcl::VFHSignature308> desc2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    for (int i = 0; i < this->pc1.planes.size(); i++)
    {
        pcl::PCLPointCloud2* temp1 = new pcl::PCLPointCloud2; 
        pcl::PCLPointCloud2Ptr temp2;
        pcl_conversions::toPCL(this->pc1.planes[i], *temp1);
        temp2 = pcl::PCLPointCloud2Ptr(temp1);
        convert(temp2, cloud);
        desc1.points.push_back(this->descriptor(cloud, this->normals(cloud, 0.03))->points[0]);
    }
    // for (int i = 0; i < this->pc2.planes.size(); i++)
    // {
    //     sensor_msgs::PointCloud2 plane = this->pc2.planes[i];
    //     convert(plane, cloud);
    //     desc1.points.push_back(*(this->descriptor(cloud, this->normals(cloud))).points[0]);
    // }
    ROS_INFO_STREAM(desc1.points.size() << "\t" << desc2.points.size());
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