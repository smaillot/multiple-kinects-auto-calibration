#include "Merging.h"

using namespace std;
using namespace message_filters;


/*
* @bief Colorize a set of planes given their correspondences.
*/
pcPtr colorize(vector<pcPtr> input, pcl::CorrespondencesPtr matches, bool match)
{
    pcPtr cloudPtr(new pc_t);
    pcPtr plane(new pc_t);
    int current_index;
    for (int i = 0; i < matches->size(); i++)
    {
        if (match) current_index = (*matches)[i].index_match;
        else current_index = (*matches)[i].index_query;
        *plane = *input[current_index];
        colorize(plane, (float)i / (float)matches->size());
        if (i == 0) *cloudPtr = *plane;
        else *cloudPtr += *plane;
    }
    return cloudPtr;
}

/*
* @bief Merge and colorize 2 sets of planes given their correspondences.
*/
pcPtr colorize(calib::PlaneClouds input1, calib::PlaneClouds input2, pcl::CorrespondencesPtr matches)
{
    vector<pcPtr> planes1;
    for (int i = 0; i < input1.planes.size(); i++)
    {
        pcPtr temp(new pc_t);
        MSGtoPCL(input1.planes[i], temp);
        planes1.push_back(temp);
    }
    vector<pcPtr> planes2;
    for (int i = 0; i < input2.planes.size(); i++)
    {
        pcPtr temp(new pc_t);
        MSGtoPCL(input2.planes[i], temp);
        planes2.push_back(temp);
    }
    pcPtr cloudPtr(new pc_t);
    *cloudPtr = *colorize(planes1, matches, false);
    *cloudPtr += *colorize(planes2, matches, true);
    return cloudPtr;
}

/*
* @bief Merge and colorize a set of keypoints given their correspondences.
*/
pcPtr colorize(pcPtr input1, pcPtr input2, pcl::CorrespondencesPtr matches)
{
    pcPtr cloudPtr(new pc_t);
    vector<int> current_index;
    for (int i = 0; i < matches->size(); i++)
    {
        current_index.clear();
        current_index.push_back((*matches)[i].index_query);
        pcPtr kp1(new pc_t(*input1, current_index));
        colorize(kp1, (float)i / (float)matches->size());
        if (i == 0) *cloudPtr = *kp1;
        else *cloudPtr += *kp1; 

        current_index.clear();
        current_index.push_back((*matches)[i].index_match);
        pcPtr kp2(new pc_t(*input2, current_index));
        colorize(kp2, (float)i / (float)matches->size());
        *cloudPtr += *kp2; 
    }
    return cloudPtr;
}

/*
* @brief Constructor
*/
Merging::Merging(ros::NodeHandle* node, string name1, string name2)
{
    this->name1 = name1;
    this->name2 = name2;
    this->node = node;

    this->radius = 0.01;    
    this->kp_dupl_rej = true;
    this->kp_est_radius = 200;

    for (int i = 0; i < 4; i++) this->callback[i] = false;

    this->sub_pc1 = this->node->subscribe("/calib/clouds/" + name1 + "/planes", 1, &Merging::pc1_callback, this);
    this->sub_pc2 = this->node->subscribe("/calib/clouds/" + name2 + "/planes", 1, &Merging::pc2_callback, this);
    this->sub_coef1 = this->node->subscribe("/calib/planes/" + name1, 1, &Merging::coef1_callback, this);
    this->sub_coef2 = this->node->subscribe("/calib/planes/" + name2, 1, &Merging::coef2_callback, this);
    this->sub_full_pc1 = this->node->subscribe("/calib/clouds/" + name1, 1, &Merging::full_pc1_callback, this);
    this->sub_full_pc2 = this->node->subscribe("/calib/clouds/" + name2, 1, &Merging::full_pc2_callback, this);
    this->pub_color = this->node->advertise<pc_t>("/calib/clouds/" + name1 + "_" + name2 + "/planes_color", 1);
    this->pub_kp = this->node->advertise<pc_t>("/calib/clouds/" + name1 + "_" + name2 + "/keypoints", 1);

	pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    this->kdtree = tree;
}

/*
* @brief Dynamic reconfiguer callback.
*/
void Merging::conf_callback(calib::MergingConfig &config, uint32_t level)
{
    // transform
    this->radius = config.normal_radius / 1000;
    this->kp_dupl_rej = config.kp_dupl_rej;
    this->kp_est_radius = config.kp_est_radius / 1000;
    this->iss_support_radius = config.iss_support_radius / 1000;
    this->iss_nms_radius = config.iss_nms_radius / 1000;
    this->match_th = config.match_th;

    this->param_cut.x.enable = config.x_enable;
    this->param_cut.x.bounds[0] = config.x_min / 1000;
    this->param_cut.x.bounds[1] = config.x_max / 1000;
    this->param_cut.y.enable = config.y_enable;
    this->param_cut.y.bounds[0] = config.y_min / 1000;
    this->param_cut.y.bounds[1] = config.y_max / 1000;
    this->param_cut.z.enable = config.z_enable;
    this->param_cut.z.bounds[0] = config.z_min / 1000;
    this->param_cut.z.bounds[1] = config.z_max / 1000;
}

void Merging::pc1_callback(const calib::PlaneCloudsConstPtr& input)
{
    this->callback[0] = true;
    this->pc1 = *input;
    if (this->isSync())
    {
        this->callback_sync();
    }
}
void Merging::pc2_callback(const calib::PlaneCloudsConstPtr& input)
{
    this->callback[1] = true;
    this->pc2 = *input;
    if (this->isSync())
    {
        this->callback_sync();
    }
}
void Merging::coef1_callback(const calib::PlanesConstPtr& input)
{
    this->callback[2] = true;
    this->coef1 = *input;
    if (this->isSync())
    {
        this->callback_sync();
    }
}
void Merging::coef2_callback(const calib::PlanesConstPtr& input)
{
    this->callback[3] = true;
    this->coef2 = *input;
    if (this->isSync())
    {
        this->callback_sync();
    }
}
void Merging::full_pc1_callback(const pcConstPtr& input)
{
    this->callback[4] = true;
    pcPtr cloud(new pc_t(*input));
    this->full_pc1 = cloud;
    if (this->isSync())
    {
        this->callback_sync();
    }
}
void Merging::full_pc2_callback(const pcConstPtr& input)
{
    this->callback[5] = true;
    pcPtr cloud(new pc_t(*input));
    this->full_pc2 = cloud;
    if (this->isSync())
    {
        this->callback_sync();
    }
}

bool Merging::isSync()
{
    if (this->callback[0] == true && this->callback[1] == true && this->callback[2] == true && this->callback[3] == true && this->callback[4] == true && this->callback[5] == true)
    {
        for (int i = 0; i < 6; i++) this->callback[i] = false;
        return true;
    }
    else
    {
        return false;
    }
}

void Merging::callback_sync()
{
    pcl::CorrespondencesPtr planes_corr(new pcl::Correspondences);
    planes_corr = this->get_corr();
    this->pub_color.publish(colorize(this->pc1, this->pc2, planes_corr));

    this->keypoints1 = remove_nans(this->extract_kp(this->full_pc1));
    this->keypoints2 = remove_nans(this->extract_kp(this->full_pc2));

    pcl::CorrespondencesPtr kp_corr(new pcl::Correspondences);
    kp_corr = this->get_kp_corr();
    this->pub_kp.publish(colorize(this->keypoints1, this->keypoints2, kp_corr));  
}

/*
* @brief Compute point cloud normals.
*
* @param input Point cloud.
* @param radius Search radius for normal estimation.
*/
pc_nPtr Merging::normals(pcPtr input, float radius)
{
    pc_nPtr normals(new pc_n_t);
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
pc_featPtr Merging::descriptor(pcPtr input, pc_nPtr normals)
{
    pc_featPtr descriptor(new plane_feat_t);
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
pcl::CorrespondencesPtr Merging::match_planes(pc_featPtr desc1, pc_featPtr desc2)
{
    if (desc1->points.size() * desc2->points.size() != 0)
    {
        pcl::CorrespondencesPtr corr(new pcl::Correspondences());
        this->corr_est.setInputSource(desc1);
        this->corr_est.setInputTarget(desc2);
        this->corr_est.determineCorrespondences(*corr);

        // this->corr_rej.setInputCorrespondences(corr);
        // this->corr_rej.getCorrespondences(*corr);

        ROS_DEBUG_STREAM("Planes (" << desc1->points.size() << "," << desc2->points.size() << ") correspondances (" << corr->size() << "):");
        for (int i = 0; i < corr->size(); i++)
        {
            ROS_DEBUG_STREAM("\tPlane" << (*corr)[i].index_query+1 << " -> Plane" << (*corr)[i].index_match+1 << " (" << (*corr)[i].distance << ")");
        }

        return corr;
    }
}

pcl::CorrespondencesPtr Merging::get_corr()
{
    pc_featPtr desc1(new plane_feat_t);
    pc_featPtr desc2(new plane_feat_t);
    pcPtr cloud(new pc_t);
    for (int i = 0; i < this->pc1.planes.size(); i++)
    {
        pc_msg_t plane_msg = this->pc1.planes[i];
        MSGtoPCL(plane_msg, cloud);
        pc_featPtr featPtr = this->descriptor(cloud, this->normals(cloud, this->radius));
        if (i == 0)
        {
            *desc1 = *featPtr;
        }
        else
        {
            *desc1 += *featPtr;
        }
    }
    for (int i = 0; i < this->pc2.planes.size(); i++)
    {
        pc_msg_t plane_msg = this->pc2.planes[i];
        MSGtoPCL(plane_msg, cloud);
        pc_featPtr featPtr = this->descriptor(cloud, this->normals(cloud, this->radius));
        if (i == 0)
        {
            *desc2 = *featPtr;
        }
        else
        {
            *desc2 += *featPtr;
        }
    }
    pcl::CorrespondencesPtr corr(new pcl::Correspondences);
    corr = match_planes(desc1, desc2);
    return corr;
}

kp_featPtr Merging::feat_est(pcPtr cloudPtr, pc_nPtr normalsPtr)
{
    kp_featPtr feat(new kp_feat_t);

    this->kp_feat_est.setSearchMethod(this->kdtree);
    this->kp_feat_est.setInputCloud(cloudPtr);
    this->kp_feat_est.setInputNormals(normalsPtr);
    this->kp_feat_est.setRadiusSearch(this->kp_est_radius);
    this->kp_feat_est.compute(*feat);

    return feat;
}

pcl::CorrespondencesPtr Merging::compute_kp_corr(kp_featPtr feat1, kp_featPtr feat2)
{
    pcl::registration::CorrespondenceEstimation<kp_t, kp_t> est;
    pcl::CorrespondencesPtr corr(new pcl::Correspondences());
    est.setInputSource(feat1);
    est.setInputTarget(feat2);
    est.determineCorrespondences(*corr);

    if (this->kp_dupl_rej)
    {
        pcl::CorrespondencesPtr corr_dupl_rej(new pcl::Correspondences());
        this->corr_rej.setInputCorrespondences(corr);
        this->corr_rej.getCorrespondences(*corr);
    }

    ROS_DEBUG_STREAM("Keypoints (" << feat1->points.size() << "," << feat2->points.size() << ") correspondances (" << corr->size() << "):");
    for (int i = 0; i < corr->size(); i++)
    {
        ROS_DEBUG_STREAM("\tKeypoint" << (*corr)[i].index_query+1 << " -> Keypoint" << (*corr)[i].index_match+1 << " (" << (*corr)[i].distance << ")");
    }

    return corr;
}

pcl::CorrespondencesPtr Merging::get_kp_corr()
{
    kp_featPtr kp1(new kp_feat_t);
    kp_featPtr kp2(new kp_feat_t);
    pc_nPtr normals1(new pc_n_t);
    pc_nPtr normals2(new pc_n_t);
    normals1 = this->normals(this->keypoints1, this->radius);
    normals2 = this->normals(this->keypoints2, this->radius);
    kp1 = feat_est(this->keypoints1, normals1);
    kp2 = feat_est(this->keypoints2, normals2);
    pcl::CorrespondencesPtr kp_corr;
    kp_corr = this->compute_kp_corr(kp1, kp2);

    pcl::CorrespondencesPtr kp_corr_filt(new pcl::Correspondences);
    for (int i = 0; i < kp_corr->size(); i++)
    {
        if ((*kp_corr)[i].distance <= this->match_th)
        {
            kp_corr_filt->push_back((*kp_corr)[i]);
        }
    }

    return kp_corr_filt;
}

pcPtr Merging::extract_kp(pcPtr cloudPtr)
{
    pcPtr cutted(new pc_t);
    pcPtr kp(new pc_t);

    cutted = this->cut(cloudPtr, this->param_cut);

    this->iss.setSalientRadius(this->iss_support_radius);
    this->iss.setNonMaxRadius(this->iss_nms_radius);
    this->iss.setInputCloud(cutted);
    this->iss.compute(*kp);
    return kp;
}

/**
 * @brief Perform cutting of the point cloud.
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr Merging::cut(pcPtr input, param_cut_t params)
{
    if (this->param_cut.x.enable || this->param_cut.y.enable || this->param_cut.z.enable)
    {
        pcPtr temp(new pc_t(*input)); 
        pcPtr filtered(new pc_t); 
        if (this->param_cut.x.enable) 
        {
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("x"); 
            this->filter_cut.setFilterLimits(this->param_cut.x.bounds[0], this->param_cut.x.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (this->param_cut.y.enable) 
        { 
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("y"); 
            this->filter_cut.setFilterLimits(this->param_cut.y.bounds[0], this->param_cut.y.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (this->param_cut.z.enable) 
        { 
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("z"); 
            this->filter_cut.setFilterLimits(this->param_cut.z.bounds[0], this->param_cut.z.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered;
        }
        return temp;
    }
    else
    {
        return input;
    }
}

pcPtr Merging::remove_nans(pcPtr cloudNans)
{
    pcPtr cloud(new pc_t);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices); 
    pcl::removeNaNFromPointCloud(*cloudNans, indices->indices); 
    pcl::ExtractIndices<Point> rm_nan; 
    rm_nan.setInputCloud(cloudNans); 
    rm_nan.setIndices(indices); 
    rm_nan.setNegative(false); 
    rm_nan.filter(*cloud);

    return cloud; 
}