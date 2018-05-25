#include "Matching.h"

using namespace std;
using namespace message_filters;


void convert(pc_msg_t input, sensor_msgs::PointCloud2ConstPtr output)
{
    sensor_msgs::PointCloud2ConstPtr temp(new pc_msg_t(input));
    output = temp;
    ROS_DEBUG_STREAM("Conversion to msgPtr, " << output->data.size() << " points.");
}

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const pc_msgPtr& input, pcl::PCLPointCloud2Ptr& output)
{
    pcl::PCLPointCloud2 temp;
    ROS_INFO_STREAM(input->data.size() << " input points"); 
    pcl_conversions::toPCL(*input, temp);
    // pcl_conversions::toPCL(input->header, temp.header);
    ROS_DEBUG_STREAM("Conversion to pclPC2, " << temp.data.size() << " points.");
    pcl::PCLPointCloud2Ptr temp2(new pcl::PCLPointCloud2(temp)); 
    output = temp2;
    ROS_DEBUG_STREAM("Conversion to pclPC2Ptr, " << output->data.size() << " points.");
} 

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const pcl::PCLPointCloud2Ptr& input, pcPtr& output)
{
    pcl::fromPCLPointCloud2(*input, *output);   
    ROS_DEBUG_STREAM("Conversion to pcPtr, " << output->points.size() << " points."); 
}

/*
* @brief Convert point cloud between pcl and sensor_msg objects.
*
* @param input Input point cloud.
* @param output Output point cloud.
*/
void convert(const pc_msgPtr& input, pcPtr& output)
{
    pcl::PCLPointCloud2Ptr temp(new pcl::PCLPointCloud2);
    convert(input, temp);
    convert(temp, output);
} 

void convert(pc_msg_t input, pcPtr output)
{
    pc_msgPtr temp(new pc_msg_t);
    convert(input, temp);
    convert(temp, output);
}

void convert(pc_msg_t input, pc_msgPtr output)
{
    pc_msgPtr temp(new pc_msg_t(input));
    *output = *temp;
    ROS_DEBUG_STREAM("Conversion to msgptr, " << output->data.size() << " points.");
}

/*
* @bief Colorize a set of planes given their correspondences.
*/
void colorize(vector<pcPtr> input, pcl::CorrespondencesPtr matches, bool match, pcPtr output)
{
    ROS_INFO_STREAM(input.size() << " planes to colorize");
    pc_t cloud;
    pc_t plane;
    int current_index;
    for (int i = 0; i < matches->size(); i++)
    {
        if (match) current_index = (*matches)[i].index_match;
        else current_index = (*matches)[i].index_query;
        plane = *input[current_index];
        pcPtr planePtr(new pc_t(plane));
        colorize(planePtr, (float)i / (float)matches->size());
        if (i == 0) cloud = plane;
        else cloud += plane;
        ROS_DEBUG_STREAM("Total points: " << cloud.points.size());
    }
    *output = cloud;
}

/*
* @bief Merge and colorize 2 sets of planes given their correspondences.
*/
void colorize(calib::Planes input1, calib::Planes input2, pcl::CorrespondencesPtr matches, pcPtr output)
{
    ROS_INFO_STREAM(matches->size() << " correspondances to colorize");
    vector<pcPtr> planes1;
    for (int i = 0; i < input1.clouds.size(); i++)
    {
        pcPtr temp(new pc_t);
        MSGtoPCL(input1.clouds[i], temp);
        ROS_DEBUG_STREAM("Adding " << temp->points.size() << " points plane");
        planes1.push_back(temp);
    }
    vector<pcPtr> planes2;
    for (int i = 0; i < input2.clouds.size(); i++)
    {
        pcPtr temp(new pc_t);
        MSGtoPCL(input2.clouds[i], temp);
        ROS_DEBUG_STREAM("Adding " << temp->points.size() << " points plane");
        planes2.push_back(temp);
    }
    pcPtr cloud1Ptr(new pc_t);
    pcPtr cloud2Ptr(new pc_t);
    colorize(planes1, matches, false, cloud1Ptr);
    colorize(planes2, matches, true, cloud2Ptr);
    *output = *cloud1Ptr + *cloud2Ptr;
}

/*
* @bief Merge and colorize a set of keypoints given their correspondences.
*/
void colorize(pcPtr& input1, pcPtr& input2, pcl::CorrespondencesPtr& matches, pcPtr& output)
{
    pc_t cloud;
    vector<int> current_index;
    ROS_DEBUG_STREAM("Colorizing " << matches->size() << " matching pairs.");
    for (int i = 0; i < matches->size(); i++)
    {
        current_index.clear();
        current_index.push_back((*matches)[i].index_query);
        pcPtr kp1(new pc_t(*input1, current_index));
        colorize(kp1, (float)i / (float)matches->size());
        if (i == 0) cloud = *kp1;
        else cloud += *kp1; 

        current_index.clear();
        current_index.push_back((*matches)[i].index_match);
        pcPtr kp2(new pc_t(*input2, current_index));
        colorize(kp2, (float)i / (float)matches->size());
        cloud += *kp2; 
    }
    *output = cloud;
}

int* range(int n)
{
    const int N = n;
    int* array = new int[N];
    for (int i = 0; i < N; i++)
    {
        array[i] = i;
    }
    return array;
}

pcl::CorrespondencesPtr array2corr(int a[], int n)
{
    pcl::CorrespondencesPtr corr(new pcl::Correspondences);
    for (int i = 0; i < n; i++)
    {   
        corr->push_back(pcl::Correspondence(i, a[i], 0));
    }
    return corr;
}

void publish(ros::Publisher& pub, pc_t& cloud)
{
    pc_msg_t msg;
    pcl::toROSMsg(cloud, msg);
    publish(pub, msg);
}

void publish(ros::Publisher& pub, const pc_msg_t& input)
{
    pc_msg_t msg = input;
    ROS_DEBUG_STREAM(msg.data.size() << " points to publish on " << pub.getTopic());
    if (msg.data.size() > 0)
    {
        pub.publish(msg);
    }
}

/*
* @brief Constructor
*/
Matching::Matching(ros::NodeHandle* node, std::string name1, std::string name2)
{
    this->name1 = name1;
    this->name2 = name2;
    this->node = node; 

    this->radius = 0.01;    
    this->kp_dupl_rej = true;
    this->kp_est_radius = 200;

    this->pub_color = this->node->advertise<pc_t>("/calib/planes/" + name1 + "_" + name2 + "/planes_color", 1);
    this->pub_kp = this->node->advertise<pc_t>("/calib/clouds/" + name1 + "_" + name2 + "/keypoints_matches", 1);
    this->pub_all_kp = this->node->advertise<pc_t>("/calib/clouds/" + name1 + "_" + name2 + "/keypoints", 1);

	pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    this->kdtree = tree;
}

void Matching::update(const calib::PlanesConstPtr& planes1, const calib::PlanesConstPtr& planes2)
{
    this->planes1 = *planes1;
    this->planes2 = *planes2;
    this->callback_sync();
}

/*
* @brief Dynamic reconfiguer callback.
*/
void Matching::conf_callback(calib::MatchingConfig &config, uint32_t level)
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

void Matching::callback_sync()
{
    pcl::CorrespondencesPtr planes_corr(new pcl::Correspondences);
    int n = this->planes1.clouds.size();
    planes_corr = array2corr(range(n), n);//this->get_corr();
    pcPtr msg(new pc_t);
    colorize(this->planes1, this->planes2, planes_corr, msg);
    publish(pub_color, *msg);

    pcPtr cloudPtr1(new pc_t);
    pcPtr cloudPtr2(new pc_t);
    pc_msg_t temp1 = this->planes1.point_cloud;
    pc_msg_t temp2 = this->planes2.point_cloud;
    convert(temp1, cloudPtr1);
    convert(temp2, cloudPtr2); 
    ROS_DEBUG_STREAM(cloudPtr1->points.size() << " points");
    ROS_DEBUG_STREAM(cloudPtr2->points.size() << " points");

    this->keypoints1 = remove_nans(this->extract_kp(cloudPtr1));
    this->keypoints2 = remove_nans(this->extract_kp(cloudPtr2));

    pcPtr kp(this->keypoints1);
    pcPtr kp2(this->keypoints2);
    colorize(kp, 0.25);
    colorize(kp2, 0.75);
    *kp += *kp2;
    publish(pub_all_kp, *kp);

    pcl::CorrespondencesPtr kp_corr(new pcl::Correspondences);
    // n = min(this->keypoints1->size(), this->keypoints2->size());
    //kp_corr = array2corr(range(n), n);
    kp_corr = this->get_kp_corr();
    pcPtr msg2(new pc_t);
    colorize(this->keypoints1, this->keypoints2, kp_corr, msg2);
    publish(pub_kp, *msg2);
}

/*
* @brief Compute point cloud normals.
*
* @param input Point cloud.
* @param radius Search radius for normal estimation.
*/
pc_nPtr Matching::normals(pcPtr input, float radius)
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
pc_featPtr Matching::descriptor(pcPtr input, pc_nPtr normals)
{
    pc_featPtr descriptor(new plane_feat_t);
    this->global_est.setInputCloud(input);
	this->global_est.setInputNormals(normals);
	this->global_est.setSearchMethod(this->kdtree);
    this->global_est.setNormalizeBins(true);
    this->global_est.setNormalizeDistance(false);
    this->global_est.compute(*descriptor);
    return descriptor;
}

/*
* @brief Match detected planes 
*
* @param desc1 Point cloud containing descriptor for each plane of the first cloud.
* @param desc2 Point cloud containing descriptor for each plane of the second cloud.
*/
pcl::CorrespondencesPtr Matching::match_planes(pc_featPtr desc1, pc_featPtr desc2)
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

pcl::CorrespondencesPtr Matching::get_corr()
{
    pc_featPtr desc1(new plane_feat_t);
    pc_featPtr desc2(new plane_feat_t);
    pcPtr cloud(new pc_t);
    for (int i = 0; i < this->planes1.clouds.size(); i++)
    {
        pc_msg_t plane_msg = this->planes1.clouds[i];
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
    for (int i = 0; i < this->planes2.clouds.size(); i++)
    {
        pc_msg_t plane_msg = this->planes2.clouds[i];
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

kp_featPtr Matching::feat_est(pcPtr cloudPtr, pc_nPtr normalsPtr)
{
    kp_featPtr feat(new kp_feat_t);

    this->kp_feat_est.setSearchMethod(this->kdtree);
    this->kp_feat_est.setInputCloud(cloudPtr);
    this->kp_feat_est.setInputNormals(normalsPtr);
    this->kp_feat_est.setRadiusSearch(this->kp_est_radius);
    this->kp_feat_est.compute(*feat);

    return feat;
}

pcl::CorrespondencesPtr Matching::compute_kp_corr(kp_featPtr feat1, kp_featPtr feat2)
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

pcl::CorrespondencesPtr Matching::get_kp_corr()
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

pcPtr Matching::extract_kp(pcPtr cloudPtr)
{
    pcPtr cutted(new pc_t);
    pcPtr kp(new pc_t);

    cutted = this->cut(cloudPtr, this->param_cut);

    this->iss.setSalientRadius(this->iss_support_radius);
    this->iss.setNonMaxRadius(this->iss_nms_radius);
    this->iss.setInputCloud(cutted);
    this->iss.compute(*kp);
    ROS_DEBUG_STREAM(kp->points.size() << " remaining points after cutting to extract keypoints");
    return kp;
}

/**
 * @brief Perform cutting of the point cloud.
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr Matching::cut(pcPtr input, param_cut_t params)
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

pcPtr Matching::remove_nans(pcPtr cloudNans)
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