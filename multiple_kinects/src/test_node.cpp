#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

tf::TransformListener *listener;
ros::Publisher pub;

double subsize;

PointCloud2 subsample_pc(const PointCloud2 pc)
{
    PointCloud2 output;

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(pc, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (subsize, subsize, subsize);
    sor.filter(filtered);

    // Convert to ROS data type
    pcl_conversions::fromPCL(filtered, output);

    return output;
}

PointCloud2 merge_pc(const PointCloud2 pc1, const PointCloud2 pc2)
{
    // PointCloud2 input1 = pc1;
    // PointCloud2 input2 = pc2;
    PointCloud2 merged_pc;
    pcl::concatenatePointCloud(pc1, pc2, merged_pc);

    return merged_pc;
}

// tf::StampedTransform get_transform(const string& cam_tf, const char* center_tf)
// {
//     tf::TransformListener listener;
//     tf::StampedTransform transform;
//     try{
//       listener.lookupTransform(cam_tf, center_tf,
//                             ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//       ros::Duration(1.0).sleep();
//     }
//
//     return transform;
// }

void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    PointCloud2 merged_pc;
    PointCloud2 filtered_pc;
    PointCloud2 input1 = *pc1;
    PointCloud2 input2 = *pc2;

    // rought pc transform
    // tf::StampedTransform transform_cam1;
    // tf::StampedTransform transform_cam2;
    // transform_cam1 = get_transform("cam1_link","cam-center");
    // transform_cam2 = get_transform("cam2_link","cam-center");

    string target_tf = "cam1_link";
    pcl_ros::transformPointCloud(target_tf, input1, input1, *listener);
    pcl_ros::transformPointCloud(target_tf, input2, input2, *listener);

    // accurate calibration
    // TODO

    // merging
    merged_pc = merge_pc(input1, input2);

    // subsampling
    if (subsize > 0 || subsize == true)
    {
        if (subsize == true) subsize = 0.01;
        filtered_pc = subsample_pc(merged_pc);
    }
    else
    {
        filtered_pc = merged_pc;
    }

    pub.publish(filtered_pc);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    listener = new tf::TransformListener();
    nh.getParam("subsample_size", subsize);

    // Synchronize both kinects messages
    message_filters::Subscriber<PointCloud2> cam1(nh, "/cam1/qhd/points", 1);
    message_filters::Subscriber<PointCloud2> cam2(nh, "/cam2/qhd/points", 1);
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
    Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
    sync.registerCallback(boost::bind(&pc_callback, _1, _2));

    pub = nh.advertise<PointCloud2>("output", 1);

    // Spin
    ros::spin();
}
