#include <plane_detection/plane_detection.h>

using namespace std;

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = 1;// sizeof(inputs) / sizeof(*inputs);
const string pc_topic_name = "/qhd/points";
const string pub_topic_name = "/reconstruction/point_clouds";

std::string get_topic_name(int input_number)
{
	return inputs[input_number] + pc_topic_name;
}

std::string get_publish_name(int input_number)
{
	return pub_topic_name + inputs[input_number];
}

int get_input_number(std::string topic_name)
{
	for (int i = 0 ; i < n_inputs ; i++)
	{
		if (get_topic_name(i) == topic_name)
		{
			return i;
		}
	}
	return -1;
}

int main(int argc, char *argv[])
{
	// verbosity: debug
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "plane_detection_node");
		ros::NodeHandle n;

		geometry::PointCloud PC0(n, get_topic_name(0), get_publish_name(0));//[n_inputs];
		geometry::PointCloud PC1(n, get_topic_name(1), get_publish_name(1));//[n_inputs];

		ros::Rate loop_rate(1);
 		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		// ros::spin();
	// }

	return 0;
}