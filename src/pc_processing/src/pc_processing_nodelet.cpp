#include "pc_processing/pc_processing.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

tf::TransformListener *listener;
ros::Publisher pub;
PcProcessing PC_object;

//  parameters
//      subsampling
double subsize;
//      filtering
bool filtering;
double filter_radius;
int filter_min_neighbors;
//      cutting
bool cutting_x_enable;
float cutting_x_min;
float cutting_x_max;
bool cutting_y_enable;
float cutting_y_min;
float cutting_y_max;
bool cutting_z_enable;
float cutting_z_min;
float cutting_z_max;


void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    // merging piont clouds
    PC_object.merge_pc(pc1, pc2);

    // subsampling point cloud
    PC_object.set_subsize(subsize);
    PC_object.subsample_pc();

    // cutting point cloud
    PC_object.set_cutting_params(cutting_x_enable, cutting_x_min, cutting_x_max, cutting_y_enable, cutting_y_min, cutting_y_max, cutting_z_enable, cutting_z_min, cutting_z_max);
    PC_object.cutting_pc();

    // filtering point cloud
    PC_object.set_filtering_params(filtering, filter_radius, filter_min_neighbors);
    PC_object.filter_pc();

    PointCloud2* ptr = PC_object.get_filtered_pc() ;
    pub.publish(*ptr);
}

void dynrec_callback(pc_processing::registrationConfig &config, uint32_t level)
{
    // subsampling
    subsize = config.subsampling_enable * config.subsampling_size / 1000;

    // filtering
    filtering = config.filter_enable;
    filter_radius = config.filter_radius / 1000;
    filter_min_neighbors = config.filter_min_neighbors;

    // cutting
    cutting_x_enable = config.cutting_x_enable;
    cutting_x_min = config.cutting_x_min / 1000;
    cutting_x_max = config.cutting_x_max / 1000;
    cutting_y_enable = config.cutting_y_enable;
    cutting_y_min = config.cutting_y_min / 1000;
    cutting_y_max = config.cutting_y_max / 1000;
    cutting_z_enable = config.cutting_z_enable;
    cutting_z_min = config.cutting_z_min / 1000;
    cutting_z_max = config.cutting_z_max / 1000;

    ROS_DEBUG("Dynamic reconfigure updated !");
}

int main(int argc, char** argv)
{
    // verbosity
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Initialize ROS
    ros::init(argc, argv, "pc_processing_nodelet");
    ros::NodeHandle nh;
    listener = new tf::TransformListener();
    PC_object.set_listener(listener);

    // dynamic reconfigure
    dynamic_reconfigure::Server<pc_processing::registrationConfig> server;
    dynamic_reconfigure::Server<pc_processing::registrationConfig>::CallbackType f;
    f = boost::bind(&dynrec_callback, _1, _2);
    server.setCallback(f);

    // Synchronize both kinects messages
    message_filters::Subscriber<PointCloud2> cam1(nh, "/cam1/qhd/points", 1);
    message_filters::Subscriber<PointCloud2> cam2(nh, "/cam2/qhd/points", 1);
    typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
    Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
    sync.registerCallback(boost::bind(&pc_callback, _1, _2));

    pub = nh.advertise<PointCloud2>("/scene/qhd/points", 1);

    // Spin
    ros::spin();
}
