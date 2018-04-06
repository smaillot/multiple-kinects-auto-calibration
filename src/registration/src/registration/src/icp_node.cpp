#include <registration/icp_node.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

vector <string> inputs;
string name;
string output_name; 
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*pc1, *input1);
    pcl::fromROSMsg (*pc2, *input2);

    PointCloud2 output;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(input1);
    icp.setInputTarget(input2);

    //icp.setUseReciprocalCorrespondences(true);
    //icp.setMaxCorrespondenceDistance(0.05);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (1);

    pcl::PointCloud<pcl::PointXYZRGB> registered;
    icp.align(registered);

    pcl::toROSMsg(registered, output);

    pub_pc.publish(output);
    ROS_DEBUG_STREAM("Publish merged point cloud " + name + " (" + patch::to_string(output.data.size()) + " points)");
}

int main(int argc, char *argv[])
{
    // verbosity: debug
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
        {
            ros::console::notifyLoggerLevelsChanged();
        }

    // parsing arguments

		inputs.clear();

        string node_name = "icp";
        ROS_DEBUG_STREAM(patch::to_string(argc) + " arguments found:");
        for (int i = 0; i < argc; i++)
        {
            ROS_DEBUG_STREAM("arg" + patch::to_string(i) + " = " + argv[i]);
        }
        if (argc > 4)
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

    // Synchronize both kinects messages
        ROS_DEBUG_STREAM("Subscribe to " + get_topic_name(0));
        ROS_DEBUG_STREAM("Subscribe to " + get_topic_name(1));
        message_filters::Subscriber<PointCloud2> cam1(nh, get_topic_name(0), 1);
        message_filters::Subscriber<PointCloud2> cam2(nh, get_topic_name(1), 1);
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2));

        ROS_INFO_STREAM("Start publishing registered point cloud " + name + " from topics\n\t" + inputs[0] + " and " + inputs[1]);
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