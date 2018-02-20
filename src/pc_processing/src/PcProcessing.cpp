#include <pc_processing/pc_processing.h>

PcProcessing::PcProcessing()
{
    // parameters
        // subsampling
            this->subsize = false;
        // cutting
            this->cutting_x_enable = false;
            this->cutting_x_min = 0;
            this->cutting_x_max = 0;
            this->cutting_y_enable = false;
            this->cutting_y_min = 0;
            this->cutting_y_max = 0;
            this->cutting_z_enable = false;
            this->cutting_z_min = 0;
            this->cutting_z_max = 0;
        // radius filtering
            this->filtering = false;
            this->filter_radius = 0;
            this->filter_min_neighbors = 0;
        // plane detection
            this->plane_detection_enable = false;
            this->plane_threshold_dist = 0;
            this->plane_filtering = 0;
            this->plane_max_it = 0;
            // this->plane_axis_x = 0;
            // this->plane_axis_y = 0;
            // this->plane_axis_z = 0;
            // this->plane_angle_th = 0;

    // tf listener
    this->tf_listener = NULL;

    // point clouds
    this->full_pc = NULL;
    this->filtered_pc = NULL;
    this->segmented_pc = NULL;

    // planes
    this->plane_pc = NULL;
    tf::Quaternion Q(0, 0, 0, 1);
    this->plane_quat = Q;
    tf::Vector3 c(0, 0, 0);
    this->plane_center = c;
}

PcProcessing::~PcProcessing()
{
}

void PcProcessing::merge_pc(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    sensor_msgs::PointCloud2 input1 = *pc1;
    // sensor_msgs::PointCloud2 input2 = *pc2;
    sensor_msgs::PointCloud2 merged_pc = input1;

    // std::string target_tf = "cam_center";
    // pcl_ros::transformPointCloud(target_tf, input1, input1, *this->tf_listener);
    // pcl_ros::transformPointCloud(target_tf, input2, input2, *this->tf_listener);

    // pcl::concatenatePointCloud(input1, input2, merged_pc);
    sensor_msgs::PointCloud2* ptr_full(new sensor_msgs::PointCloud2(merged_pc));
    this->full_pc = ptr_full;
    sensor_msgs::PointCloud2* ptr_filt(new sensor_msgs::PointCloud2(merged_pc));
    this->filtered_pc = ptr_filt;
}

void PcProcessing::subsample_pc()
{
    if (subsize > 0 || subsize == true)
    {
        if (subsize == true) subsize = 0.01;
        // Container for original & filtered data
            pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
            pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
            pcl::PCLPointCloud2 filtered;
            sensor_msgs::PointCloud2 subsampled;

        // Convert to PCL data type
            pcl_conversions::toPCL(*this->filtered_pc, *cloud);

        // Perform the filtering
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloudPtr);
            sor.setLeafSize (this->subsize, this->subsize, this->subsize);
            sor.filter(filtered);

        // Convert to ROS data type
            pcl_conversions::fromPCL(filtered, subsampled);
            sensor_msgs::PointCloud2* ptr(new sensor_msgs::PointCloud2(subsampled));
            this->filtered_pc = ptr;
    }
}

void PcProcessing::set_subsize(double subsize)
{
    this->subsize = subsize;
}

void PcProcessing::set_filtering_params(bool filtering, double filter_radius, int filter_min_neighbors)
{
    this->filtering = filtering;
    this->filter_radius = filter_radius;
    this->filter_min_neighbors = filter_min_neighbors;
}

void PcProcessing::set_cutting_params(bool cutting_x_enable, float cutting_x_min, float cutting_x_max, bool cutting_y_enable, float cutting_y_min, float cutting_y_max, bool cutting_z_enable, float cutting_z_min, float cutting_z_max)
{
    this->cutting_x_enable = cutting_x_enable;
    this->cutting_x_min = cutting_x_min;
    this->cutting_x_max = cutting_x_max;
    this->cutting_y_enable = cutting_y_enable;
    this->cutting_y_min = cutting_y_min;
    this->cutting_y_max = cutting_y_max;
    this->cutting_z_enable = cutting_z_enable;
    this->cutting_z_min = cutting_z_min;
    this->cutting_z_max = cutting_z_max;
}

void PcProcessing::set_plane_detection_params(bool plane_detection, double dist_th, double filtering, int max_it) //, double axis_x, double axis_y, double axis_z, double angle_th)
{
    this->plane_detection_enable = plane_detection;
    this->plane_threshold_dist = dist_th;
    this->plane_filtering = filtering;
    this->plane_max_it = max_it;
    // this->plane_axis_x = plane_axis_x;
    // this->plane_axis_y = plane_axis_y;
    // this->plane_axis_z = plane_axis_z;
    // this->plane_angle_th = angle_th;
}

void PcProcessing::set_listener(const tf::TransformListener* listener)
{
    this->tf_listener = listener;
}

sensor_msgs::PointCloud2* PcProcessing::get_filtered_pc()
{
    return this->filtered_pc;
}

sensor_msgs::PointCloud2* PcProcessing::get_full_pc()
{
    return this->full_pc;
}

sensor_msgs::PointCloud2* PcProcessing::get_plane_pc()
{
    return this->plane_pc;
}

void PcProcessing::filter_pc()
{
    ROS_DEBUG("Filtering point cloud...");
    if (this->filtering)
    {
        // Container for original & filtered data
        ROS_DEBUG("\tstart filtering");
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2 filtered;
        sensor_msgs::PointCloud2 subsampled;

        // Convert to PCL data type
        ROS_DEBUG("\tconversion to pcl");
        pcl_conversions::toPCL(*this->filtered_pc, *cloud);

        // filtering
        ROS_DEBUG("\tcreating the radius neighbors filter");
        pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;

        // build the filter
        outrem.setInputCloud(cloudPtr);
        outrem.setRadiusSearch(this->filter_radius);
        outrem.setMinNeighborsInRadius(this->filter_min_neighbors);

        // apply filter
        ROS_DEBUG("\tapply radius neighbors filter");
        outrem.filter(filtered);

        // Convert to ROS data type
        ROS_DEBUG("\tconverting to PointCloud2...");
        pcl_conversions::fromPCL(filtered, subsampled);
        sensor_msgs::PointCloud2* msg(new sensor_msgs::PointCloud2(subsampled));
        this->filtered_pc = msg;
        ROS_INFO("Filtering sucessful !");
    }
    else
    {
        ROS_INFO("Filtering disabled.");
    }
}

void PcProcessing::cutting_pc()
{
    ROS_DEBUG("Cutting point cloud...");

    if (this->cutting_x_enable || this->cutting_y_enable || this->cutting_z_enable)
    {
        // Container for original & filtered data
        ROS_DEBUG("\tstart cutting");
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        sensor_msgs::PointCloud2 cut;

        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        // add conditions

        if (this->cutting_x_enable && (this->cutting_x_min) < (this->cutting_x_max))
        {
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, this->cutting_x_min)));
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, this->cutting_x_max)));
        }
        if (this->cutting_y_enable && (this->cutting_y_min) < (this->cutting_y_max))
        {
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, this->cutting_y_min)));
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, this->cutting_y_max)));
        }
        if (this->cutting_z_enable && (this->cutting_z_min) < (this->cutting_z_max))
        {
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, this->cutting_z_min)));
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, this->cutting_z_max)));
        }

        // Convert to PCL data type
        ROS_DEBUG("\tconversion to pcl");
        pcl_conversions::toPCL(*this->filtered_pc, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*cloud, *temp_cloud);

        // filtering
        ROS_DEBUG("\tcreating the condition filter");
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(temp_cloud);
        condrem.setKeepOrganized(true);

        // apply filter
        ROS_DEBUG("\tapply condition filter");
        condrem.filter(*filtered);

        // Convert to ROS data type
        ROS_DEBUG("\tconverting to PointCloud2...");
            sensor_msgs::PointCloud2* msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*filtered, *msg);
        this->filtered_pc = msg;
        ROS_INFO("Cutting sucessful !");
    }
    else
    {
        ROS_INFO("All cuttings are disabled !");
    }
}

void PcProcessing::initialize_seg_pc()
{
    this->segmented_pc = this->filtered_pc;
}

void PcProcessing::plane_detection(int plane_n)
{
    if (this->plane_detection_enable)
    {
        ROS_DEBUG("\tStart plane detection");
        pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>);

        ROS_DEBUG("\tConvert msg to PCL");
        // Convert to PCL data type
            pcl_conversions::toPCL(*this->segmented_pc, *cloud2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(*cloud2, *cloud);

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        ROS_DEBUG("\tCreate segmentation model");
        // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;

            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (this->plane_threshold_dist);
            seg.setMaxIterations(this->plane_max_it);
            // seg.setAxis(Eigen::Vector3f(this->plane_axis_x, this->plane_axis_y, this->plane_axis_z));
            // seg.setEpsAngle(this->plane_angle_th); 

            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

        ROS_DEBUG("\tCompute plane translation");
        // translation
            tf::Vector3 normal_vect(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            // double dist = std::abs(coefficients->values[3]) / (std::pow(coefficients->values[0], 2) + std::pow(coefficients->values[1], 2) + std::pow(coefficients->values[2], 2));
            Eigen::Matrix< double, 4, 1 > transl; // = -normal_vect * dist;
            pcl::compute3DCentroid(*cloud, *inliers, transl);
            tf::Vector3 translation(transl(0, 0), transl(1, 0), transl(2, 0));

        ROS_DEBUG("\tCompute plane rotation");
        // rotation
            tf::Vector3 z(0, 0, 1);
            double angle = z.angle(normal_vect);
            tf::Vector3 axis = z.cross(normal_vect);
        
        ROS_DEBUG("\tTime filtering");
        // time filtering
            translation = this->plane_filtering * this->plane_center + (1 - this->plane_filtering) * translation;
            angle = this->plane_filtering * this->plane_quat.getAngle() + (1 - this->plane_filtering) * angle;
            axis = this->plane_filtering * this->plane_quat.getAxis() + (1 - this->plane_filtering) * axis;

        ROS_DEBUG("\tCompute quaternion");
        // compute quaternion
            tf::Quaternion Q;
            Q.setRotation(axis, angle);

        ROS_DEBUG("\tConvert into tf");
        // convert into tf
            tf::Transform transform;
            transform.setOrigin(translation);
            transform.setRotation(Q);
        
        ROS_DEBUG("\tBroadcast tf");
        // broadcast
            this->plane_quat = Q;
            this->plane_center = translation;
            static tf::TransformBroadcaster br; 
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/cam_center", "/plane" + patch::to_string(plane_n))); 

        ROS_DEBUG("\tExtract inliers");
        // extract inliers
            pcl::ExtractIndices<pcl::PointXYZ> extract_pos;
            extract_pos.setInputCloud (cloud);
            extract_pos.setIndices (inliers);
            extract_pos.setNegative (false);
            extract_pos.filter (*plane);

        // subtract in segmented pc
            pcl::ExtractIndices<pcl::PointXYZ> extract_neg;
            extract_neg.setInputCloud (cloud);
            extract_neg.setIndices (inliers);
            extract_neg.setNegative (true);
            extract_neg.filter (*remaining);

        sensor_msgs::PointCloud2* msg_pos(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*plane, *msg_pos);
        this->plane_pc = msg_pos;
        sensor_msgs::PointCloud2* msg_neg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*remaining, *msg_neg);
        this->segmented_pc = msg_neg;

        Plane plane_to_save(axis, translation);
        planes.push_back(plane_to_save);
    }
    else
    {
        ROS_DEBUG("Plane detection is disabled");
    }
}

void PcProcessing::line_detection()
{
    int n = this->planes.size();

    for (int i = 0 ; i < n ; i++)
    {
        for  (int j = 0 ; j < i ; j++)
        {
            lines.push_back(this->planes[i].intersect(this->planes[j]));
        }
    }
}