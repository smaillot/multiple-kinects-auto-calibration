#include <merging_node.h>

using namespace std;
using namespace message_filters;

vector <string> inputs;
string name;
ros::Publisher pub_pc;
string output;

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback(const pcConstPtr& pc1, const pcConstPtr& pc2)
{
    string frame = inputs[0] + "_" + inputs[1];
    string pub_frame;
    tf::StampedTransform transform;
    tf::TransformListener tf_listener;

    pc_t input1 = *pc1;
    pc_t input2 = *pc2;
    ROS_DEBUG_STREAM("Receiving pc1 in " << input1.header.frame_id);
    ROS_DEBUG_STREAM("Receiving pc2 in " << input2.header.frame_id);
    ros::param::param<string>("/calib/" + inputs[1] + "/cloud/frame_pub", pub_frame, "world");

    try
    {
        tf_listener.waitForTransform(frame, pub_frame, ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform(frame, pub_frame, ros::Time(0), transform);
        pcl_ros::transformPointCloud(input1, input1, transform);
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
    pc_t merged = input1 + input2;
    ROS_DEBUG_STREAM("merged pc in " << merged.header.frame_id);
    pub_pc.publish(merged);
}

int main(int argc, char *argv[])
{

    // parsing arguments

		inputs.clear();

        inputs.push_back(argv[1]);
        inputs.push_back(argv[2]);
        output = argv[3];
        string node_name = "merging_" + output;

    // Initialize ROS
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;

        pub_pc = nh.advertise<pc_msg_t>("/calib/clouds/" + output, 1);
        // ros::Duration(30.0).sleep();

    // Synchronize both kinects messages
        message_filters::Subscriber<pc_t> cam1_sub(nh, "/calib/clouds/" + inputs[0] + "/preproc", 1);
        message_filters::Subscriber<pc_t> cam2_sub(nh, "/calib/clouds/" + inputs[1] + "/preproc", 1);
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