#include "Matching.h"

using namespace std;
using namespace message_filters;


void convert(pc_msg_t input, sensor_msgs::PointCloud2ConstPtr output)
{
    sensor_msgs::PointCloud2ConstPtr temp(new pc_msg_t(input));
    output = temp;
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
    ROS_DEBUG_STREAM(input.size() << " planes to colorize");
    pc_t cloud;
    pc_t plane;
    int current_index;
    for (int i = 0; i < matches->size(); i++)
    {
        if (match) current_index = (*matches)[i].index_match;
        else current_index = (*matches)[i].index_query;
        plane = *input[current_index];
        pcPtr planePtr(new pc_t(plane));
        colorize(planePtr, (float)current_index / (float)matches->size());
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
    ROS_DEBUG_STREAM(matches->size() << " correspondances to colorize");
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
void colorize(pcPtr& input1, pcPtr& input2, pcl::CorrespondencesPtr& matches, pcPtr& output1, pcPtr& output2)
{
    pc_t cloud1;
    pc_t cloud2;
    vector<int> current_index;
    ROS_DEBUG_STREAM("Colorizing " << matches->size() << " matching pairs.");
    for (int i = 0; i < matches->size(); i++)
    {
        current_index.clear();
        current_index.push_back((*matches)[i].index_query);
        pcPtr kp1(new pc_t(*input1, current_index));
        colorize(kp1, (float)i / (float)matches->size());
        // colorize(kp1, 0);
        if (i == 0) cloud1 = *kp1;
        else cloud1 += *kp1; 

        current_index.clear();
        current_index.push_back((*matches)[i].index_match);
        pcPtr kp2(new pc_t(*input2, current_index));
        colorize(kp2, (float)i / (float)matches->size());
        // colorize(kp2, 0.5);
        if (i == 0) cloud2 = *kp1;
        else cloud2 += *kp2; 
    }
    *output1 = cloud1;
    *output2 = cloud2;
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
    this->pub_matches = this->node->advertise<calib::Matches>("/calib/" + name1 + "_" + name2 + "/matches", 1);
    this->pub_objects1 = this->node->advertise<pc_t>("/calib/clouds/" + name1 + "/objects", 1);
    this->pub_objects2 = this->node->advertise<pc_t>("/calib/clouds/" + name2 + "/objects", 1);

	pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    this->kdtree = tree;

    this->kp_type = 1;
    this->cut_reverse = false;
    this->match_th = 1.0;
    this->kp_est_radius = 0.5;
    this->radius = 0.05;
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
    this->kp_dupl_rej = config.kp_corr_rej;
    this->match_th_dist = config.match_th_dist / 1000;
    if (config.extract_objects)
    {
        this->cut_th = 0.1;
    }
    else
    {
        this->cut_th = -0.1;
    }
}

void Matching::callback_sync()
{
    pcl::CorrespondencesPtr planes_corr(new pcl::Correspondences);
    int n = this->planes1.clouds.size();
    planes_corr = this->get_corr();
    // planes_corr = array2corr(range(n), n);
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

    this->cloud1 = cloudPtr1;
    this->cloud2 = cloudPtr2;
    this->keypoints1 = remove_nans(this->extract_kp(cloudPtr1, this->planes1, this->pub_objects1));
    this->keypoints2 = remove_nans(this->extract_kp(cloudPtr2, this->planes2, this->pub_objects2));

    pcPtr kp(new pc_t(*this->keypoints1));
    pcPtr kp2(new pc_t(*this->keypoints2));
    colorize(kp, 0.25);
    colorize(kp2, 0.75);
    *kp += *kp2;
    publish(this->pub_all_kp, *kp);

    pcl::CorrespondencesPtr kp_corr(new pcl::Correspondences);
    // n = min(this->keypoints1->size(), this->keypoints2->size());
    //kp_corr = array2corr(range(n), n);
    kp_corr = this->get_kp_corr();
    pcPtr msg2_1(new pc_t);
    pcPtr msg2_2(new pc_t);
    colorize(this->keypoints1, this->keypoints2, kp_corr, msg2_1, msg2_2);
    pc_t pc = *msg2_1 + *msg2_2;
    publish(this->pub_kp, pc);

    calib::Matches matches;
    matches.header = this->planes1.header;
    shape_msgs::Plane p1;
    shape_msgs::Plane p2;
    for (int i = 0; i < planes_corr->size(); i++)
    {
        p1 = this->planes1.planes[(*planes_corr)[i].index_query];
        p2 = this->planes2.planes[(*planes_corr)[i].index_match];
        matches.planes1.push_back(p1);
        matches.planes2.push_back(p2);
    }

    pc_msg_t points_msg1;
    pc_msg_t points_msg2;
    pcl::toROSMsg(*msg2_1, points_msg1);
    pcl::toROSMsg(*msg2_2, points_msg2);
    matches.points1 = points_msg1;
    matches.points2 = points_msg2;

    this->pub_matches.publish(matches);
}

/*
* @brief Compute point cloud normals.
*
* @param input Point cloud.
* @param radius Search radius for normal estimation.
*/
pc_nPtr Matching::normals(pcPtr input, float radius)
{
    ROS_DEBUG_STREAM("Compute normals of " << input->points.size() << " points.");
    pc_nPtr normals(new pc_n_t);
    this->norm.setInputCloud(input);
    this->norm.setRadiusSearch(radius);
    this->norm.setSearchMethod(this->kdtree);
    this->norm.compute(*normals);
    ROS_DEBUG_STREAM("\t-> " << normals->points.size() << " normals.");
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
    // Global plane descriptor to match planes
    ///////////////////////
    // pc_featPtr desc1(new plane_feat_t);
    // pc_featPtr desc2(new plane_feat_t);
    // pcPtr cloud(new pc_t);
    // for (int i = 0; i < this->planes1.clouds.size(); i++)
    // {
    //     pc_msg_t plane_msg = this->planes1.clouds[i];
    //     MSGtoPCL(plane_msg, cloud);
    //     pc_featPtr featPtr = this->descriptor(cloud, this->normals(cloud, this->radius));
    //     if (i == 0)
    //     {
    //         *desc1 = *featPtr;
    //     }
    //     else
    //     {
    //         *desc1 += *featPtr;
    //     }
    // }
    // for (int i = 0; i < this->planes2.clouds.size(); i++)
    // {
    //     pc_msg_t plane_msg = this->planes2.clouds[i];
    //     MSGtoPCL(plane_msg, cloud);
    //     pc_featPtr featPtr = this->descriptor(cloud, this->normals(cloud, this->radius));
    //     if (i == 0)
    //     {
    //         *desc2 = *featPtr;
    //     }
    //     else
    //     {
    //         *desc2 += *featPtr;
    //     }
    // }
    // pcl::CorrespondencesPtr corr(new pcl::Correspondences);
    // corr = match_planes(desc1, desc2);
    // return corr;

    // Match by normals similarity (assume pc orientations are similar)
    //////////////////////
    pcl::CorrespondencesPtr corr(new pcl::Correspondences);
    for (int i = 0; i < this->planes1.planes.size(); i++)
    {
        pcl::Correspondence c;
        int best = -1;
        float best_score = -99;
        c.index_query = i;
        shape_msgs::Plane p1 = this->planes1.planes[i];

        for (int j = 0; j < this->planes2.planes.size(); j++)
        {
            shape_msgs::Plane p2 = this->planes2.planes[j];
            float score = p1.coef[0] * p2.coef[0] + p1.coef[1] * p2.coef[1] + p1.coef[2] * p2.coef[2] - 0.1 * abs(p1.coef[3] - p2.coef[3]);
            ROS_DEBUG_STREAM("Match plane " << i+1 << ":" << p1.coef[0] << ", " << p1.coef[1] << ", " << p1.coef[2] << " with plane " << j+1 << ": " << p2.coef[0] << ", " << p2.coef[1] << ", " << p2.coef[2] << "\n\tcost = " << score);
            if (score > best_score && score > -99)
            {
                best_score = score;
                best = j;
                ROS_DEBUG_STREAM("new best !");
            }
        }
        if (best != -1)
        {
            ROS_INFO_STREAM("Matching planes: " << i+1 << " -> " << best+1);
            c.index_match = best;
            corr->push_back(c);
        }
    }

    return corr;
}

kp_featPtr Matching::feat_est(pcPtr& cloudPtr, pc_nPtr& normalsPtr, pcPtr& surfacePtr)
{
    kp_featPtr feat(new kp_feat_t);

    // this->kp_feat_est.setSearchMethod(this->kdtree);
    this->kp_feat_est.setInputCloud(cloudPtr);
    this->kp_feat_est.setInputNormals(normalsPtr);
    this->kp_feat_est.setSearchSurface(surfacePtr);
    this->kp_feat_est.setRadiusSearch(this->kp_est_radius);
    this->kp_feat_est.compute(*feat);

    return feat;
}

pcl::CorrespondencesPtr Matching::compute_kp_corr(kp_featPtr feat1, kp_featPtr feat2)
{
    pcl::CorrespondencesPtr corr(new pcl::Correspondences());
    // pcl::registration::CorrespondenceEstimation<kp_t, kp_t> est;
    // est.setInputSource(feat1);
    // est.setInputTarget(feat2);
    // est.determineCorrespondences(*corr);

    
    this->match_search.setInputCloud(feat1);
    for (size_t i = 0; i < feat2->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!pcl_isfinite (feat2->at(i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = this->match_search.nearestKSearch(feat2->at(i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1) 
        {
            pcl::Correspondence c (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            corr->push_back(c);
        }
    }

    if (this->kp_dupl_rej)
    {
        pcl::CorrespondencesPtr corr_dupl_rej(new pcl::Correspondences());
        this->corr_rej.setInputSource(this->cloud1);
        this->corr_rej.setInputTarget(this->cloud2);
        this->corr_rej.setInputCorrespondences(corr);
        this->corr_rej.getCorrespondences(*corr);
    }

    ROS_DEBUG_STREAM("Keypoints (" << feat1->points.size() << "," << feat2->points.size() << ") correspondances (" << corr->size() << "):");
    for (int i = 0; i < corr->size(); i++)
    {
        ROS_DEBUG_STREAM("\tKeypoint" << (*corr)[i].index_query+1 << " -> Keypoint" << (*corr)[i].index_match+1 << " (" << (*corr)[i].distance << ")");
    }

    ROS_INFO_STREAM(corr->size() << " keypoints matches.");
    return corr;
}

pcl::CorrespondencesPtr Matching::get_kp_corr()
{
    kp_featPtr kp1(new kp_feat_t);
    kp_featPtr kp2(new kp_feat_t);
    pc_nPtr normals1(new pc_n_t);
    pc_nPtr normals2(new pc_n_t);
    normals1 = this->normals(this->cloud1, this->radius);
    normals2 = this->normals(this->cloud2, this->radius);
    kp1 = feat_est(this->keypoints1, normals1, this->cloud1);
    kp2 = feat_est(this->keypoints2, normals2, this->cloud2);
    pcl::CorrespondencesPtr kp_corr;
    kp_corr = this->compute_kp_corr(kp1, kp2);

    pcl::CorrespondencesPtr kp_corr_filt(new pcl::Correspondences);
    for (int i = 0; i < kp_corr->size(); i++)
    {
        if ((*kp_corr)[i].distance <= this->match_th)
        {
            int a = (*kp_corr)[i].index_query;
            int b = (*kp_corr)[i].index_match;
            Eigen::Vector3f v(this->keypoints1->points[a].x, this->keypoints1->points[a].y, this->keypoints1->points[a].z);
            Eigen::Vector3f w(this->keypoints2->points[b].x, this->keypoints2->points[b].y, this->keypoints2->points[b].z);
            float dist = sqrt((v - w).squaredNorm());
            if (dist <= this->match_th_dist)
            {
                ROS_DEBUG_STREAM("Keeping match with distance " << dist << "m (" << v[0] << ", " << v[1] << ", " << v[2] << " - " << w[0] << ", " << w[1] << ", " << w[2] << ")");
                kp_corr_filt->push_back((*kp_corr)[i]);
            }
            else
            {
                ROS_DEBUG_STREAM("Reject match with distance " << dist << "m (" << v[0] << ", " << v[1] << ", " << v[2] << " - " << w[0] << ", " << w[1] << ", " << w[2] << ")");
            }
        }
    }

    return kp_corr_filt;
}

pcPtr Matching::extract_kp(pcPtr cloudPtr, calib::Planes planes, ros::Publisher pub)
{
    pcPtr cutted(new pc_t);
    pcPtr kp(new pc_t);

    cutted = this->cut_plane(cloudPtr, planes);
    pub.publish(*cutted);
    // cutted = this->cut(cloudPtr, this->param_cut);

    double res1;
    double res2;
    this->node->param<double>("/calib/" + name1 + "/preproc/size", res1, 20);    
    this->node->param<double>("/calib/" + name2 + "/preproc/size", res2, 20);        
    double resolution = min(res1, res2) / 1000;
    this->iss_support_radius = resolution;
    this->iss_nms_radius = resolution; 
    ROS_DEBUG_STREAM("iss kp size = " << resolution);

    if (this->kp_type == 1 && this->iss_support_radius * this->iss_nms_radius != 0)
    {
        this->iss.setSalientRadius(this->iss_support_radius);
        this->iss.setNonMaxRadius(this->iss_nms_radius);
        this->iss.setInputCloud(cutted);
        this->iss.compute(*kp);
        ROS_DEBUG_STREAM(kp->points.size() << " remaining points after cutting to extract keypoints");
    }
    else if(this->kp_type == 2)
    {
        pcl::SIFTKeypoint<Point, pcl::PointWithScale> sift_detect;
        pcl::PointCloud<pcl::PointWithScale> result;

        sift_detect.setSearchMethod(pcl::search::KdTree<Point>::Ptr (new pcl::search::KdTree<Point>));
        sift_detect.setScales(this->min_scale, this->nr_octaves, this->nr_scales_per_octave);
        sift_detect.setMinimumContrast(this->min_contrast);
        sift_detect.setInputCloud(cutted);
        sift_detect.compute(result);
        pcl::copyPointCloud(result, *kp);
        kp->header.frame_id = cloudPtr->header.frame_id;
    }
    else
    {
        kp = cutted;
    }


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
    if (params.x.enable || params.y.enable || params.z.enable)
    {
        pcPtr temp(new pc_t(*input)); 
        pcPtr filtered(new pc_t); 
        if (params.x.enable) 
        {
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("x"); 
            this->filter_cut.setFilterLimits(params.x.bounds[0], params.x.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (params.y.enable) 
        { 
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("y"); 
            this->filter_cut.setFilterLimits(params.y.bounds[0], params.y.bounds[1]); 
            this->filter_cut.filter(*filtered);
            temp = filtered; 
        } 
        if (params.z.enable) 
        { 
            this->filter_cut.setInputCloud(temp); 
            this->filter_cut.setFilterFieldName("z"); 
            this->filter_cut.setFilterLimits(params.z.bounds[0], params.z.bounds[1]); 
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

/**
 * @brief Perform cutting of the point cloud given a plane equation.
 * 
 * @param input Input cloud.
 * @param params Filter parameters.
 */
pcPtr Matching::cut_plane(pcPtr input, calib::Planes planes)
{
    pcPtr cloudPtr(new pc_t(*input));

    for (int i = 0; i < planes.planes.size(); i++)
    {
        shape_msgs::Plane plane = planes.planes[i];
        ROS_DEBUG_STREAM("Cutting in normal direction: " << plane.coef[0] << ", " << plane.coef[1] << ", " << plane.coef[2] << ", " << plane.coef[3]);
        // build a tf which z axis is the plane normal
        //     tf::Vector3 normal;                   
        //     tf::Vector3 point;
        //     tf::Vector3 z(0,0,1);
        //     tf::Vector3 axis;
        //     tf::Transform transform;
        //     normal = tf::Vector3(plane.coef[0], plane.coef[1], plane.coef[2]).normalized();
        //     point = - normal * std::abs(plane.coef[3]) / normal.length();
        //     ROS_DEBUG_STREAM("from: " << point.getX() << ", " << point.getY() << ", " << point.getZ());
        //     transform.setOrigin(point);
        //     axis = z.cross(normal);

        //     // tf::Vector3 a, b(1,1,-(normal.getX() + normal.getY())/normal.getZ());
        //     // b.normalize();
        //     // a = b.cross(normal);
        //     // tf::Matrix3x3 mat(a.getX(), b.getX(), normal.getX(), a.getY(), b.getY(), normal.getY(), a.getZ(), b.getZ(), normal.getZ());
        //     // double r, p, y;
        //     // mat.getRPY(r, p, y);
        //     // transform.setRotation(tf::Quaternion(r, p, y));

        //     float w = sqrt(((pow(z.length(), 2)) * (pow(normal.length(), 2))) + z.dot(normal));
        //     transform.setRotation(tf::Quaternion(axis.getX(), axis.getY(), axis.getZ(), w));
        //     transform = transform.inverse();
        //     if (this->cut_reverse)
        //     {
        //         transform = transform.inverse();
        //     }

        // // transform point cloud and cut in the z direction
        //     pcl_ros::transformPointCloud(*cloudPtr, *cloudPtr, transform);  

        //     param_cut_t params;
        //     params.x.enable = false;
        //     params.x.bounds[0] = 0;
        //     params.x.bounds[1] = 0;
        //     params.y.enable = false;
        //     params.y.bounds[0] = 0;
        //     params.y.bounds[1] = 0;
        //     params.z.enable = true;
        //     params.z.bounds[0] = this->cut_th;
        //     params.z.bounds[1] = 5;
        //     cloudPtr = this->cut(cloudPtr, params);

        // // get back to the initial frame
        //     pcl_ros::transformPointCloud(*cloudPtr, *cloudPtr, transform.inverse());  


        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        for (int i = 0; i < cloudPtr->points.size(); i++)
        {
            tf::Vector3 point(cloudPtr->points[i].x, cloudPtr->points[i].y, cloudPtr->points[i].z);
            if (i % 1000 == 0)
            {
                ROS_DEBUG_STREAM("Point " << point.getX() << ", " << point.getY() << ", " << point.getZ() << " with value: " << (point.getX() * plane.coef[0] + point.getY() * plane.coef[1] + point.getZ() * plane.coef[2] + plane.coef[3]));
            }
            bool side = (point.getX() * plane.coef[0] + point.getY() * plane.coef[1] + point.getZ() * plane.coef[2] + plane.coef[3] > this->cut_th);
            if ((side && !this->cut_reverse) || (!side && this->cut_reverse))
            {
                inliers->indices.push_back(i);
            }
        }
        ROS_DEBUG_STREAM("Extracted points: " << inliers->indices.size());
        pcl::ExtractIndices<Point> extract;
        extract.setInputCloud(cloudPtr);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloudPtr);
        ROS_DEBUG_STREAM(cloudPtr->points.size() << " points remaining in point cloud.");
    }

    return cloudPtr;
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