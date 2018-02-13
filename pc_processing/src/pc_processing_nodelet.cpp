#include "pc_processing/pc_processing.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

tf::TransformListener *listener;
ros::Publisher pub;
pc_processing PC_object;
double subsize;

void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    PC_object.merge_pc(pc1, pc2);
    PC_object.set_subsize(0.02);
    PC_object.subsample_pc();

    pub.publish(*PC_object.get_subsampled_pc());
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
    nh.getParam("pc_subsampling_size", subsize);
    listener = new tf::TransformListener();
    PC_object.set_listener(listener);

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
