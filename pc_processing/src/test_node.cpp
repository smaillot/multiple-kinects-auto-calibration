#include "pc_processing/test_node.h"

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

void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
// clock_t begin_time = clock();

    PointCloud2 merged_pc;
    PointCloud2 filtered_pc;
    PointCloud2 input1 = *pc1;
    PointCloud2 input2 = *pc2;

// clock_t init_time = clock();

    string target_tf = "cam_center";
    pcl_ros::transformPointCloud(target_tf, input1, input1, *listener);
    pcl_ros::transformPointCloud(target_tf, input2, input2, *listener);

// clock_t transform_time = clock();

    // accurate calibration
    // TODO

    // merging
    pcl::concatenatePointCloud(input1, input2, merged_pc);

// clock_t merge_time = clock();

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

// clock_t filter_time = clock();

    pub.publish(filtered_pc);

// clock_t end_time = clock();

    // cout << "Total time: " << double(end_time - begin_time) / 1000 << endl;
    // cout << "Init time: " << double(init_time - begin_time) / 1000 << endl;
    // cout << "Transform time: " << double(transform_time - init_time) / 1000 << endl;
    // cout << "Merge time: " << double(merge_time - transform_time) / 1000 << endl;
    // cout << "Filtering time: " << double(filter_time - merge_time) / 1000 << endl;
    // cout << "Publishing time: " << double(end_time - filter_time) / 1000 << endl;
    // cout << "\n\n\n";
}

void dynrec_callback(multiple_kinects::subsamplingConfig &config, uint32_t level)
{
    subsize = config.subsampling * config.size / 100;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    listener = new tf::TransformListener();
    nh.getParam("subsample_size", subsize);

    // dynamic reconfigure
    dynamic_reconfigure::Server<multiple_kinects::subsamplingConfig> server;
    dynamic_reconfigure::Server<multiple_kinects::subsamplingConfig>::CallbackType f;
    f = boost::bind(&dynrec_callback, _1, _2);
    server.setCallback(f);

    // Synchronize both kinects messages
    message_filters::Subscriber<PointCloud2> cam1(nh, "/cam1/qhd/points", 1);
    message_filters::Subscriber<PointCloud2> cam2(nh, "/cam2/qhd/points", 1);
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
    Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
    sync.registerCallback(boost::bind(&pc_callback, _1, _2));

    pub = nh.advertise<PointCloud2>("/kinects/qhd/points", 1);

    // Spin
    ros::spin();
}
