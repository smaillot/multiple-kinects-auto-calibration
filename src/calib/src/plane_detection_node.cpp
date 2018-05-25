#include "plane_detection_node.h"

using namespace std;

float freq = 0;

int main(int argc, char *argv[])
{
///////////
// init
    string name = argv[1];

    string node_name = (string) "plane_detection_" + name;
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_planes("/calib/" + name + "/planes");

///////////
// main code

	PlaneDetector planes(&node_planes, name);
	dynamic_reconfigure::Server<calib::PlanesConfig> server_planes(node_planes);
	dynamic_reconfigure::Server<calib::PlanesConfig>::CallbackType f_planes;
	f_planes = boost::bind(&PlaneDetector::conf_callback, &planes, _1, _2);
	server_planes.setCallback(f_planes);

////////////
// ros loop
	while (ros::ok())
	{
		ros::spinOnce();
		if (freq > 0)
		{
			ros::Rate loop_rate(freq);
			loop_rate.sleep();
		}
	}

	return 0;
///////////
}