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
const string topic_namespace = "/reconstruction/point_clouds";
string frame[2];
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
    PointCloud2 merged_pc;
    try
    {
        pcl_ros::transformPointCloud(frame[0], input1, input1, *listener);
        pcl_ros::transformPointCloud(frame[1], input2, input2, *listener);
    }
    catch (...)
    {
    	ROS_WARN("Error while transforming point cloud");
    }
    pcl::concatenatePointCloud(input1, input2, merged_pc);

    pub_pc.publish(merged_pc);
}

void dynrec_callback(registration::MergingConfig &config, uint32_t level)
{
    frame[0] = config.frame1;
    frame[1] = config.frame2;
}

int main(int argc, char *argv[])
{
    // verbosity: debug
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
        {
            ros::console::notifyLoggerLevelsChanged();
        }

    // parsing arguments

		// TODO error catching 

		inputs.clear();
        string name = argv[1];
        string node_name = "merging_" + name;
		int i = 2;
		while (i < argc-1) 
		{
			inputs.push_back(argv[i]);
			i++;
		}
        output_name = argv[argc-1];

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
        message_filters::Subscriber<PointCloud2> cam1(nh, get_topic_name(0), 1);
        message_filters::Subscriber<PointCloud2> cam2(nh, get_topic_name(1), 1);
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2));

        
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