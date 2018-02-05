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

sensor_msgs::PointCloud2 subsample_pc(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 filtered;
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (subsize, subsize, subsize);
    sor.filter(filtered);

    // Convert to ROS data type
    pcl_conversions::fromPCL(filtered, output);

    return output;
}

void pc_callback(const PointCloud2ConstPtr& pc1)//, const PointCloud2ConstPtr& pc2)
{
    PointCloud2 filt_pc1;
    //PointCloud2 filt_pc2;
    pcl::PCLPointCloud2* cloud1 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud1);
    pcl_conversions::toPCL(*pc1, *cloud1);

    filt_pc1 = subsample_pc(cloudPtr);
    pub.publish(filt_pc1);
}

int
main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Subscriber<PointCloud2> cam1(nh, "/cam1/qhd/points", 1);
    // Subscriber<PointCloud2> cam2(nh, "/cam2/qhd/points", 1);
    // TimeSynchronizer<PointCloud2, PointCloud2> sync(cam1, cam2, 10);
    // sync.registerCallback(boost::bind(&pc_callback, _1, _2));
    //typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPC;
    //Synchronizer<SyncPC> sync(SyncPC(10), cam1, cam1);
    //sync.registerCallback(boost::bind(&pc_callback, _1, _2));
    //cam1.registerCallback(pc_callback);

    ros::Subscriber sub = nh.subscribe ("/cam1/qhd/points", 1, pc_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<PointCloud2>("output", 1);

    // Spin
    ros::spin();
}
