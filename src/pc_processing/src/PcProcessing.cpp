#include <pc_processing/pc_processing.h>

PcProcessing::PcProcessing()
{
    // parameters
    this->subsize = false;

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

double PcProcessing::get_subsize()
{
    return this->subsize;
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
        std::clock_t start;
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
        ROS_DEBUG("\tcreating the filter");
        pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;

        // build the filter
        outrem.setInputCloud(cloudPtr);
        outrem.setRadiusSearch(this->filter_radius);
        outrem.setMinNeighborsInRadius(this->filter_min_neighbors);

        // apply filter
        ROS_DEBUG("\tapply filter");
        outrem.filter(filtered);

        // Convert to ROS data type
        ROS_DEBUG("\tconverting to PointCloud2...");
        pcl_conversions::fromPCL(filtered, subsampled);
        sensor_msgs::PointCloud2* ptr(new sensor_msgs::PointCloud2(subsampled));
        this->filtered_pc = ptr;
        ROS_INFO("\tFiltering sucessful !");
    }
    else
    {
        ROS_INFO("Filtering disabled.");
    }
}