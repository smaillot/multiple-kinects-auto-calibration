#include <registration/merging_node.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
/*
*	args parser
*
*	node frequency
*	input topics namespace
*	output topics namespace
*/

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string sub_topic_name = "/reconstruction/point_clouds";
const string pub_topic_name = "/reconstruction/point_clouds/merged";
float frequency = 2;
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
	ROS_DEBUG_STREAM(sub_topic_name + inputs[input_number]);
	return sub_topic_name + inputs[input_number];
}

/**
 * @brief Callback from kinects synchronization.
 */
void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2)
{
    PointCloud2 input1 = *pc1;
    PointCloud2 input2 = *pc2;
    PointCloud2 merged_pc;
    std::string target_tf = "cam_center2";
    pcl_ros::transformPointCloud(target_tf, input2, input2, *listener);
    pcl::concatenatePointCloud(input1, input2, merged_pc);

    pub_pc.publish(merged_pc);
}

int main(int argc, char *argv[])
{
	// verbosity: debug
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "merging_nodes");
        ros::NodeHandle nh;

    // Synchronize both kinects messages
        message_filters::Subscriber<PointCloud2> cam1(nh, get_topic_name(0), 1);
        message_filters::Subscriber<PointCloud2> cam2(nh, get_topic_name(1), 1);
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2));

        
        pub_pc = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1);

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