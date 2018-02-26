#include <registration/plane_detection_node.h>

using namespace std;

/*
*	args parser
*/

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string sub_topic_name = "/reconstruction/point_clouds";
const string pub_topic_name = "/reconstruction/planes";
int frequency = 0;
vector<geometry::PointCloud*> PC;

std::string get_topic_name(int input_number)
{
	return sub_topic_name + inputs[input_number];
}

std::string get_publish_name()
{
	return pub_topic_name;
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
		ros::NodeHandle nh;

	// create PointCloud objects
		for (int i = 0 ; i < n_inputs ; i++)
		{
			PC.push_back(new geometry::PointCloud(nh, get_topic_name(i), get_publish_name()));
		}

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