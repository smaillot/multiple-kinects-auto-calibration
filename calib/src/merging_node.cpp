#include <merging_node.h>

using namespace std;
using namespace message_filters;

vector <string> inputs;
string name;
ros::Publisher pub_pc;
string output;
pcl::PassThrough<Point> filter_cut;
string frame_proc = "cam_center";
bool enable_cutting;
float y_pose;
float width;
bool icp = false;

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback(const pcConstPtr& pc1, const pcConstPtr& pc2)
{
    string frame = inputs[0] + "_" + inputs[1] + "_filtered";
    string pub_frame;
    tf::StampedTransform transform;
    tf::StampedTransform transform2;
    tf::TransformListener tf_listener;

    pc_t input1 = *pc1;
    pc_t input2 = *pc2;
    ROS_DEBUG_STREAM("Receiving pc1 in " << input1.header.frame_id);
    ROS_DEBUG_STREAM("Receiving pc2 in " << input2.header.frame_id);
    if (icp)
    {
        pub_frame = name + "_filtered";
    }
    else
    {
        ros::param::param<string>("/calib/" + inputs[1] + "/cloud/frame_pub", pub_frame, "world");
    }
    // ros::param::param<string>("/calib/" + inputs[0] + "_" + inputs[1] + "/transform_estimation/frame", frame_proc, "cam_center");

    try
    {
        tf_listener.waitForTransform(input1.header.frame_id, frame_proc, ros::Time(0), ros::Duration(5.0));
        pcl_ros::transformPointCloud(frame_proc, input1, input1, tf_listener);
        tf_listener.waitForTransform(input2.header.frame_id, frame_proc, ros::Time(0), ros::Duration(5.0));
        pcl_ros::transformPointCloud(frame_proc, input2, input2, tf_listener);

        tf_listener.waitForTransform(frame, frame_proc, ros::Time(0), ros::Duration(5.0));
        tf_listener.lookupTransform(frame, frame_proc, ros::Time(0), transform);
        // transform2.setRotation(transform.getRotation());
        // tf_listener.lookupTransform(frame_proc, frame, ros::Time(0), transform);
        // transform2.setOrigin(transform.getOrigin());
        pcl_ros::transformPointCloud(input1, input1, transform);

        if (enable_cutting)
        {
            cut(input1, y_pose-width/2, 5); 
            cut(input2, -5, y_pose+width/2); 
        }

        tf_listener.waitForTransform(pub_frame, frame_proc, ros::Time(0), ros::Duration(5.0));
        pcl_ros::transformPointCloud(pub_frame, input1, input1, tf_listener);
        pcl_ros::transformPointCloud(pub_frame, input2, input2, tf_listener);
        ROS_DEBUG_STREAM("pc1 now in " << input1.header.frame_id);
        ROS_DEBUG_STREAM("pc2 now in " << input2.header.frame_id);
    }
    catch (tf::TransformException ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    ROS_DEBUG("Merging.");
    pc_t merged;
    if (icp)
    {
        merged = input1;
    }
    else
    {
        merged = input1 + input2;
    }
    ROS_DEBUG_STREAM("merged pc in " << merged.header.frame_id);
    pub_pc.publish(merged);
}

void conf_callback(calib::MergingConfig &config, uint32_t level)
{
    enable_cutting = config.enable_cutting;
    y_pose = config.y_pose / 1000;
    width = config.width / 1000;
}

void cut(pc_t& input, float ymin, float ymax)
{ 
    { 
        filter_cut.setInputCloud(pcConstPtr(new pc_t(input))); 
        filter_cut.setFilterFieldName("y"); 
        filter_cut.setFilterLimits(ymin, ymax); 
        // filter_cut.setFilterLimitsNegative(true);
        filter_cut.filter(input);
    } 
}

int main(int argc, char *argv[])
{

    // parsing arguments

		inputs.clear();

        inputs.push_back(argv[1]);
        if (argv[2] == "icp")
        {
            inputs.push_back(inputs[0]);
            icp = true;
            name = inputs[0] + "_icp";
        }
        else
        {
            inputs.push_back(argv[2]);
            name = inputs[0] + "_" + inputs[1];
        }
        output = argv[3];
        string node_name = "merging_" + output;

    // Initialize ROS
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;
        ros::NodeHandle node_merge("/calib/" + name + "/merging");

        pub_pc = nh.advertise<pc_msg_t>("/calib/clouds/" + output, 1);

        dynamic_reconfigure::Server<calib::MergingConfig> server(node_merge);
        dynamic_reconfigure::Server<calib::MergingConfig>::CallbackType f;
        f = boost::bind(&conf_callback, _1, _2);
        server.setCallback(f);        

    // Synchronize both kinects messages
        message_filters::Subscriber<pc_t> cam1_sub(nh, "/calib/clouds/" + inputs[0], 1);
        message_filters::Subscriber<pc_t> cam2_sub(nh, "/calib/clouds/" + inputs[1], 1);
        typedef sync_policies::ApproximateTime<pc_t, pc_t> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1_sub, cam2_sub);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2));


    // ROS loop
    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}