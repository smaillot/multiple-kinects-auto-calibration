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

string topic; 
string name;
//string frame;
const string pub_topic_name = "/reconstruction/planes";
float frequency = 1;
geometry::PlaneDetector* PD;

/**
 * @brief Returns the name of the topic to publish lpanes in.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_publish_name()
{
	return pub_topic_name + "/" + name;
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

	PD->set_params(enabled, n_planes, th_dist, max_it);

    ROS_DEBUG("Plane detection parameters config updated");
}

int main(int argc, char *argv[])
{

	// parsing arguments

		// TODO error catching 
		
		name = argv[1];
		topic = argv[2];
		//frame = argv[3];

	// Initialize ROS
		string node_name = "plane_detection_";
		node_name += name;
		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;
		ros::NodeHandle node_ransac("plane_detection/" + name);
		ros::Duration(0.5).sleep();

	// create PlaneDetector object
		PD = new geometry::PlaneDetector(nh, topic, get_publish_name(), "cam_center");//, frame);

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