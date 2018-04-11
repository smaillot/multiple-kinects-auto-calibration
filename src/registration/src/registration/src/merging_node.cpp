#include <registration/merging_node.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
/*
*   args parser
*
*   node frequency
*   input topics namespace
*   output topics namespace
*/

vector <string> inputs; // {"/reconstruction/point_clouds/cam1", "/reconstruction/point_clouds/cam2"};
//const int n_inputs = sizeof(inputs) / sizeof(*inputs);
string frame[2];
string name;
string output_name; // = "fix";
float frequency = 0;
ros::Publisher pub_pc;
tf::TransformListener *listener;
tf::StampedTransform transf; 

/**
 * @brief Returns the publishing topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_topic_name(int input_number)
{
    return inputs[input_number];
}

std::string get_publish_name()
{
	return output_name;
}

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    PointCloud2 input1 = *pc1;
    PointCloud2 input2 = *pc2;
    PointCloud2 output1 = *pc1;
    PointCloud2 output2 = *pc2;
    PointCloud2 merged_pc;
    // try
    // {    
    //     listener->lookupTransform(frame[0], pc1->header.frame_id, ros::Time::now(), transf);
    //     pcl_ros::transformPointCloud(frame[0], transf, input1, output1);
    // }
    // catch (tf::TransformException ex)
    // {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }
    // try
    // {
    //     listener->waitForTransform(frame[1], "cam_center", ros::Time::now(), ros::Duration(10.0));
    //     listener->lookupTransform(frame[1], "cam_center", ros::Time::now(), transf);
    //     pcl_ros::transformPointCloud(frame[1], transf, input2, output2);
    // }
    // catch (tf::TransformException ex)
    // {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }
     
    pcl::concatenatePointCloud(output1, output2, merged_pc);

    pub_pc.publish(merged_pc);
    ROS_DEBUG_STREAM("Publish merged point cloud " + name + " (" + patch::to_string(merged_pc.data.size()) + " points)");
}

void dynrec_callback(registration::MergingConfig &config, uint32_t level)
{
    frame[0] = config.frame1;
    frame[1] = config.frame2;
}

int main(int argc, char *argv[])
{

    // parsing arguments

		inputs.clear();

        string node_name = "merging";
        ROS_DEBUG_STREAM(patch::to_string(argc) + " arguments found:");
        for (int i = 0; i < argc; i++)
        {
            ROS_DEBUG_STREAM("arg" + patch::to_string(i) + " = " + argv[i]);
        }
        if (argc >= 4+2)
        {
            name = argv[1];
            node_name += "_" + name;
            inputs.push_back(argv[2]);
            inputs.push_back(argv[3]);
            output_name = argv[4];
        }
        else
        {
            ROS_FATAL("4 arguments are needed\nusage:\tnew_name\tinput topic 1\tinput topic 2\toutput topic");
            return 1;
        }

    // Initialize ROS
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;
        ros::NodeHandle node_config("postprocessing/" + name + "/merging");
    
    // dynamic reconfigure
        dynamic_reconfigure::Server<registration::MergingConfig> server(node_config);
        dynamic_reconfigure::Server<registration::MergingConfig>::CallbackType f;
        f = boost::bind(&dynrec_callback, _1, _2);
        server.setCallback(f);

    // Synchronize both kinects messages
        ROS_DEBUG_STREAM("Subscribe to " + get_topic_name(0));
        ROS_DEBUG_STREAM("Subscribe to " + get_topic_name(1));
        message_filters::Subscriber<PointCloud2> cam1(nh, get_topic_name(0), 1);
        message_filters::Subscriber<PointCloud2> cam2(nh, get_topic_name(1), 1);
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2));

        ROS_INFO_STREAM("Start publishing merged point cloud " + name + " from topics\n\t" + inputs[0] + " and " + inputs[1]);
        pub_pc = nh.advertise<sensor_msgs::PointCloud2>(get_publish_name(), 1);

    // ROS loop
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        
        // sleep
            if (frequency > 0)
            {
                loop_rate.sleep();
                ros::spinOnce();
            }
            else
            {
                ros::spin();
            }
    }

    return 0;
}