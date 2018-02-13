#include <sensor_msgs/PointCloud2.h>
#include <pc_processing/pc_processing.h>

pc_processing::pc_processing()
{
    this->subsize = false;
    this->tf_listener = NULL;
    this->full_pc = NULL;
    this->subsampled_pc = NULL;
}

pc_processing::~pc_processing()
{
}

void pc_processing::merge_pc(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    sensor_msgs::PointCloud2 input1 = *pc1;
    sensor_msgs::PointCloud2 input2 = *pc2;

    std::string target_tf = "cam_center";
    pcl_ros::transformPointCloud(target_tf, input1, input1, *this->tf_listener);
    pcl_ros::transformPointCloud(target_tf, input2, input2, *this->tf_listener);

    pcl::concatenatePointCloud(input1, input2, *(this->full_pc));
}

void pc_processing::subsample_pc()
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*this->full_pc, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (this->subsize, this->subsize, this->subsize);
    sor.filter(filtered);

    // Convert to ROS data type
    pcl_conversions::fromPCL(filtered, *this->subsampled_pc);
}

void pc_processing::set_subsize(double subsize)
{
    this->subsize = subsize;
}

double pc_processing::get_subsize()
{
    return this->subsize;
}

void pc_processing::set_listener(const tf::TransformListener* listener)
{
    this->tf_listener = listener;
}

sensor_msgs::PointCloud2* pc_processing::get_subsampled_pc()
{
    return this->subsampled_pc;
}