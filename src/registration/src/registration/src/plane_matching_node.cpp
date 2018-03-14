#include <registration/plane_matching_node.h>

using namespace std;
/*
*	args parser
*
*	node frequency
*   input list
*	input topics namespace
*	output topics namespace
*/

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string sub_topic_name = "/reconstruction/planes";
const string pub_topic_name = "/reconstruction/lines";
float frequency = 2;
vector <geometry::Plane*> temp_planes;
vector <vector <geometry::Plane*> > planes;
// vector <vector <geometry::PointCloud*> > PC[n_inputs];

/**
 * @brief Returns the publishing topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_topic_name(int input_number, int plane)
{
	return sub_topic_name + inputs[input_number] + "/plane" + patch::to_string(plane);
}

/**
 * @brief Returns the name of the topic to publish planes in.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_publish_name(int input_number, int plane_number)
{
  return pub_topic_name + inputs[input_number] + "_" + patch::to_string(plane_number) + "_registered";
}

bool tf_exists(tf::TransformListener* tf_listener, tf::StampedTransform* tf, string name)
{
	bool exists = false;
	ros::Duration(0.1).sleep();
	try
	{
		tf_listener->lookupTransform(REFERENCE_FRAME, name, ros::Time(0), *tf);
		exists = true;
	}
	catch (tf::TransformException &ex)
	{
		// ROS_DEBUG_STREAM(name << " not found");
	}
	return exists;
}

// /**
//  * @brief Callback for plane matching parameters dynamic reconfigure reconfigure.
//  */
// void plane_matching_conf_callback(plane_matching::PlaneMatchingConfig &config, uint32_t level)
// {
//     bool enabled = config.enabled;
// 	int n_planes = config.n_planes;
// 	float th_dist = config.th_dist / 1000;
// 	int max_it = config.max_it;

// 	for (int i = 0 ; i < n_inputs ; i++)
// 	{
// 		PD[i]->set_params(enabled, n_planes, th_dist, max_it);
// 	}

//     ROS_DEBUG("Plane detection parameters config updated");
// }

int main(int argc, char *argv[])
{
	// verbosity: debug
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "plane_matching_node");
		ros::NodeHandle nh;
		ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(pub_topic_name, 100);
		// ros::NodeHandle node_ransac("~/RANSAC");
    	tf::TransformListener tf_listener; 
		static tf::TransformBroadcaster br;
    	tf::StampedTransform tf; 
		ros::Duration(0.5).sleep(); // for for tf listener initialization
		bool new_planes;

	// // dynamic reconfigure
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig> plane_detection_srv(node_ransac);
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig>::CallbackType plane_detection_cb;
	// 	plane_detection_cb = boost::bind(&plane_detection_conf_callback, _1, _2);
	// 	plane_detection_srv.setCallback(plane_detection_cb);

	// ROS loop
	ros::Rate loop_rate(frequency);
	while (ros::ok())
	{
		planes.clear();
		for (int i = 0; i < n_inputs; i++)
		{
			// update planes
				temp_planes.clear();
				int n = 0;
				while (tf_exists(&tf_listener, &tf, get_topic_name(i, n+1)))
				{
					temp_planes.push_back(new geometry::Plane(tf));
					n++;
				}
				planes.push_back(temp_planes);
			// // update point clouds if needed
			// 	if (new_planes) 
			// 	{
			// 		PC[i].clear();
			// 		ROS_DEBUG_STREAM(patch::to_string(n) << " planes detected for " << inputs[i]);
			// 		for (int j = 0 ; j < n ; j++)
			// 		{
			// 			PC[i].push_back(new geometry::PointCloud(nh, get_topic_name(i, j), get_publish_name(i, j)));
			// 		}
			// 	}
		}

	// sleep
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