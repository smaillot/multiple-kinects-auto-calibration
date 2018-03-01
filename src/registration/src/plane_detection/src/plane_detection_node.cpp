#include <plane_detection/plane_detection_node.h>
using namespace std;
#include <iostream>

/*
*	args parser
*
*	node frequency
*	input list
*	number of planes
*	input topics namespace
*	input topics name
*	output topics namespace
*/

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string sub_topic_name = "/reconstruction/point_clouds";
const string pub_topic_name = "/reconstruction/planes";
float frequency = 0;
vector<geometry::PlaneDetector*> PD;


/**
 * @brief Returns the publishing topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_topic_name(int input_number)
{
	return sub_topic_name + inputs[input_number];
}

/**
 * @brief Returns the name of the topic to publish lpanes in.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_publish_name(int input_number)
{
	return pub_topic_name + inputs[input_number];
}

/**
 * @brief Callback for plane detection parameters dynamic reconfigure reconfigure.
 */
void plane_detection_conf_callback(plane_detection::PlaneDetectionConfig &config, uint32_t level)
{
    bool enabled = config.enabled;
	int n_planes = config.n_planes;
	float th_dist = config.th_dist / 1000;
	int max_it = config.max_it;

	for (int i = 0 ; i < n_inputs ; i++)
	{
		PD[i]->set_params(enabled, n_planes, th_dist, max_it);
	}

    ROS_DEBUG("Plane detection parameters config updated");
}

int main(int argc, char *argv[])
{
	// verbosity: debug
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "plane_detection_node");
		ros::NodeHandle nh;
		ros::NodeHandle node_ransac("~/RANSAC");

	// create PlaneDetector objects
		for (int i = 0 ; i < n_inputs ; i++)
		{
			PD.push_back(new geometry::PlaneDetector(nh, get_topic_name(i), get_publish_name(i)));
		}

	// dynamic reconfigure
		dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig> plane_detection_srv(node_ransac);
		dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig>::CallbackType plane_detection_cb;
		plane_detection_cb = boost::bind(&plane_detection_conf_callback, _1, _2);
		plane_detection_srv.setCallback(plane_detection_cb);

	// ROS loop
	ros::Rate loop_rate(frequency);
	while (ros::ok())
	{
		if (frequency > 0)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		else
		{
			ros::spin();
		}
	}

	return 0;
}