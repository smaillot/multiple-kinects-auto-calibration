#include <registration/line_matching_node.h>

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
 * @brief Returns the name of the topic to publish lpanes in.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_publish_name(int input_number, int line_number)
{
	return pub_topic_name + inputs[input_number] + "_" + patch::to_string(line_number);
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
    	tf::StampedTransform tf; 
		ros::Duration(0.5).sleep(); // for for tf listener initialization
		vector <geometry::Plane> cam_planes;
		vector <geometry::Line> cam_lines;
		vector <vector <geometry::Line> > lines;
		vector <int> n_lines;
		int cam_n_lines;

	// // dynamic reconfigure
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig> plane_detection_srv(node_ransac);
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig>::CallbackType plane_detection_cb;
	// 	plane_detection_cb = boost::bind(&plane_detection_conf_callback, _1, _2);
	// 	plane_detection_srv.setCallback(plane_detection_cb);

	// ROS loop
	ros::Rate loop_rate(frequency);
	while (ros::ok())
	{
		// compute planes_intersections
			lines.clear();
			n_lines.clear();
			for (int i = 0 ; i < n_inputs ; i++)
			{
				cam_planes.clear();
				cam_lines.clear();
				int n = 1;
				while (tf_exists(&tf_listener, &tf, get_topic_name(i, n)))
				{
					cam_planes.push_back(geometry::Plane(tf));
					n++;
				}
				int n_planes = n-1;
				ROS_DEBUG_STREAM(patch::to_string(n_planes) << " planes detected for " << inputs[i]);
				for (int j = 0 ; j < n_planes ; j++)
				{
					for (int k = j + 1 ; k < n_planes ; k++)
					{
						cam_lines.push_back(cam_planes[j].intersect(cam_planes[k]));
					}
				}
				lines.push_back(cam_lines);
				cam_n_lines = (n_planes * (n_planes - 1)) / 2;
				n_lines.push_back(cam_n_lines);
				ROS_DEBUG_STREAM(patch::to_string(cam_n_lines) << " intersection(s) computed for " << inputs[i]);
			}

		// list points to display
			visualization_msgs::Marker line_list;

			line_list.header.frame_id = REFERENCE_FRAME;
			line_list.header.stamp = ros::Time::now();
			line_list.action = visualization_msgs::Marker::ADD;
			line_list.pose.orientation.w = 1.0;
			line_list.type = visualization_msgs::Marker::LINE_LIST;
			line_list.scale.x = 0.01;
			line_list.color.b = 1.0;
			line_list.color.a = 1.0;

			vector <geometry_msgs::Point> line;
			for (int i = 0 ; i < n_inputs ; i++)
			{
				line_list.ns = inputs[i];
				line_list.id = i;
				line_list.points.clear();
				for (int l = 0 ; l < n_lines[i] ; l++)
				{
					line.clear();
					line = lines[i][l].get_points(5);
					ROS_DEBUG_STREAM("\t" << inputs[i] << "\tline" << patch::to_string(l+1) << ":\t(" << line[0].x << ", " << line[0].y << ", " << line[0].z << ")\t->\t(" << line[1].x << ", " << line[1].y << ", " << line[1].z << ")");
					line_list.points.insert(line_list.points.end(), line.begin(), line.end());
				}
 				marker_pub.publish(line_list);
			}

		// publish markers

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