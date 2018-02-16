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

    // tf listener
    this->tf_listener = NULL;

    // point clouds
    this->full_pc = NULL;
    this->filtered_pc = NULL;
}

PcProcessing::~PcProcessing()
{
}

void PcProcessing::merge_pc(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    sensor_msgs::PointCloud2 input1 = *pc1;
    sensor_msgs::PointCloud2 input2 = *pc2;
    sensor_msgs::PointCloud2 merged_pc;

    std::string target_tf = "cam_center";
    pcl_ros::transformPointCloud(target_tf, input1, input1, *this->tf_listener);
    pcl_ros::transformPointCloud(target_tf, input2, input2, *this->tf_listener);

    pcl::concatenatePointCloud(input1, input2, merged_pc);
    sensor_msgs::PointCloud2* ptr(new sensor_msgs::PointCloud2(merged_pc));
    this->full_pc = ptr;
    this->filtered_pc = ptr;
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
    else
    {
        this->filtered_pc = this->full_pc;
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

void PcProcessing::set_listener(const tf::TransformListener* listener)
{
    this->tf_listener = listener;
}

sensor_msgs::PointCloud2* PcProcessing::get_filtered_pc()
{
    return this->filtered_pc;
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