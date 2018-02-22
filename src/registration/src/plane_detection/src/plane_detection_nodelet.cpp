// ros
	#include <ros/console.h>
	#include "ros/ros.h"
// std
	#include <string>
// point cloud
	#include <geometry/PointCloud.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_conversions/pcl_conversions.h>

using namespace std;

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string pc_topic_name = "/qhd/points";

ros::Publisher pc_pub;

// create pc object
	tf::TransformListener* listener;
	geometry::PointCloud PC;

void pc_sub_callback(const sensor_msgs::PointCloud2ConstPtr& pc)
{
	ROS_DEBUG("Callback");
	PC.update(pc);

	sensor_msgs::PointCloud2* msg;
	msg = PC.get_pc();
	
	pc_pub.publish(*msg);
}

int main(int argc, char *argv[])
{
	// verbosity: debug
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "plane_detection_nodelet");
		ros::NodeHandle n;
		pc_pub = n.advertise<sensor_msgs::PointCloud2>("/scene/full/points", 1);

		listener = new tf::TransformListener();
		PC.init(listener);

	while (ros::ok())
	{
		std::string topic_name = inputs[0] + pc_topic_name;
		ROS_DEBUG_STREAM(topic_name);
		ros::Subscriber pc_sub = n.subscribe(topic_name, 1, pc_sub_callback);

		ros::spin();
	}

	return 0;
}