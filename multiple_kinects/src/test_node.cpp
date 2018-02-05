#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;
const double subsize = 0.01;

sensor_msgs::PointCloud2 subsample_pc(const sensor_msgs::PointCloud2ConstPtr& pc)
{
    sensor_msgs::PointCloud2 output;

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*pc, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (subsize, subsize, subsize);
    sor.filter(filtered);

    // Convert to ROS data type
    pcl_conversions::fromPCL(filtered, output);

    return output;
}

void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    PointCloud2 filt_pc;
    PointCloud2 merged_pc;

    //pcl::concatenatePointCloud(pc1, pc2, merged_pc);
    filt_pc = subsample_pc(pc1);

    pub.publish(filt_pc);
}

int
main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<PointCloud2> cam1(nh, "/cam1/qhd/points", 1);
    message_filters::Subscriber<PointCloud2> cam2(nh, "/cam2/qhd/points", 1);
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
    Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam1);
    sync.registerCallback(boost::bind(&pc_callback, _1, _2));

    pub = nh.advertise<PointCloud2>("output", 1);

    // Spin
    ros::spin();
}
