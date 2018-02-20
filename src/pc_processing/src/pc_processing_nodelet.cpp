#include "pc_processing/pc_processing.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

ros::Publisher pub_full_pc;
ros::Publisher pub_filtered_pc;
// ros::Publisher pub_table_pc;
// ros::Publisher pub_wall_pc;
ros::Publisher marker_pub;
tf::TransformListener *listener;
PcProcessing PC_object;

//  parameters
    // subsampling
        double subsize;
    // filtering
        bool filtering;
        double filter_radius;
        int filter_min_neighbors;
    // cutting
        bool cutting_x_enable;
        float cutting_x_min;
        float cutting_x_max;
        bool cutting_y_enable;
        float cutting_y_min;
        float cutting_y_max;
        bool cutting_z_enable;
        float cutting_z_min;
        float cutting_z_max;
    // plane detection
        bool plane_detection;
        double plane_dist_th;
        double plane_filtering;
        int plane_max_it;
        // double plane_axis_x;
        // double plane_axis_y;
        // double plane_axis_z;
        // double plane_angle_th;


void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    // merging piont clouds
    ROS_DEBUG("Merging point clouds...");
    PC_object.merge_pc(pc1, pc2);

    // subsampling point cloud
    ROS_DEBUG("Subsampling point clouds...");
    PC_object.set_subsize(subsize);
    PC_object.subsample_pc();

    // cutting point cloud
    ROS_DEBUG("Cutting point clouds...");
    PC_object.set_cutting_params(cutting_x_enable, cutting_x_min, cutting_x_max, cutting_y_enable, cutting_y_min, cutting_y_max, cutting_z_enable, cutting_z_min, cutting_z_max);
    PC_object.cutting_pc();

    // filtering point cloud
    ROS_DEBUG("Filtering point clouds...");
    PC_object.set_filtering_params(filtering, filter_radius, filter_min_neighbors);
    PC_object.filter_pc();

    // detect plane
        
    ROS_DEBUG("Running planes detection...");
    PC_object.set_plane_detection_params(plane_detection, plane_dist_th, filtering, plane_max_it); //, plane_axis_x, plane_axis_y, plane_axis_z, plane_angle_th);
    PC_object.initialize_seg_pc();
    for (int i = 1 ; i <= 2 ; i++)
    {
        PC_object.plane_detection(i);
    }
    PointCloud2* filtered_pc = PC_object.get_filtered_pc();
    pub_filtered_pc.publish(*filtered_pc);
    PointCloud2* full_pc = PC_object.get_full_pc();
    pub_full_pc.publish(*full_pc);

    // if (PC_object.plane_detection_enable)
    // {
    //     pub_table_pc.publish(*table_pc);
    //     pub_wall_pc.publish(*wall_pc);
    // }

    // line detection
    // PC_object.line_detection();
    // for (int i = 0 ; i < PC_object.lines.size() ; i++)
    // {
    //     visualization_msgs::Marker line_strip;
    //     line_strip.header.frame_id = "/cam_center";
    //     line_strip.header.stamp = ros::Time::now();
    //     line_strip.ns = "intersect"; 
    //     line_strip.action = visualization_msgs::Marker::ADD;
    //     line_strip.pose.orientation.w = 1.0;
    //     line_strip.id = i;
    //     line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    //     line_strip.scale.x = 0.02;
    //     line_strip.color.r = 1.0;
    //     line_strip.color.a = 1.0;

    //     geometry_msgs::Point p1;
    //     p1 = PC_object.lines[i].find_point_min();
    //     line_strip.points.push_back(p1);
    //     geometry_msgs::Point p2;
    //     p2 = PC_object.lines[i].find_point_max();
    //     line_strip.points.push_back(p2);

    //     marker_pub.publish(line_strip);
    // }
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

    // plane detection
    plane_detection = config.plane_detection_enable;
    plane_dist_th = config.plane_dist_th / 1000;
    plane_filtering = config.plane_filtering / 100;
    plane_max_it = config.plane_max_it;
    // plane_axis_x = config.plane_axis_x;
    // plane_axis_y = config.plane_axis_y;
    // plane_axis_z = config.plane_axis_z;
    // plane_angle_th = config.plane_angle_th / 180 * 3.14159;

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

    pub_filtered_pc = nh.advertise<PointCloud2>("/scene/filtered/points", 1);
    pub_full_pc = nh.advertise<PointCloud2>("/scene/full/points", 1);
    // pub_table_pc = nh.advertise<PointCloud2>("/scene/planes/table", 1);
    // pub_wall_pc = nh.advertise<PointCloud2>("/scene/planes/wall", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/scene/lines", 1);

    // Spin
    ros::spin();
}
