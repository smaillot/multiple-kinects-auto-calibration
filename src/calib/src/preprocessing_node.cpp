#include "preprocessing_node.h"

using namespace std;

float freq = 0;

void conf_callback(calib::NodeConfig &config, uint32_t level)
{
    freq = config.freq;
	switch(config.logger)
	{
		case 0:
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
			{
				ros::console::notifyLoggerLevelsChanged();
			}
			break;
		case 2:
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) 
			{
				ros::console::notifyLoggerLevelsChanged();
			}
			break;
		case 3:
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal)) 
			{
				ros::console::notifyLoggerLevelsChanged();
			}
			break;
		default:
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
			{
				ros::console::notifyLoggerLevelsChanged();
			}
	}
}

int main(int argc, char *argv[])
{
///////////
// init
    string topic = argv[1];
    string name = argv[2];

    string node_name = (string) "cloud_" + name;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("/calib/" + name);
    ros::NodeHandle node_cloud("/calib/" + name + "/cloud");
    ros::NodeHandle node_preproc("/calib/" + name + "/preproc");

    dynamic_reconfigure::Server<calib::NodeConfig> server(nh);
	dynamic_reconfigure::Server<calib::NodeConfig>::CallbackType f;
	f = boost::bind(&conf_callback, _1, _2);
	server.setCallback(f);

///////////
// main code

	Cloud cloud(&node_cloud, topic, name);
	dynamic_reconfigure::Server<calib::CloudConfig> server_cloud(node_cloud);
	dynamic_reconfigure::Server<calib::CloudConfig>::CallbackType f_cloud;
	f_cloud = boost::bind(&Cloud::conf_callback, &cloud, _1, _2);
	server_cloud.setCallback(f_cloud);

	Preprocessing preproc(&node_preproc, &cloud);
	dynamic_reconfigure::Server<calib::PreprocessingConfig> server_preproc(node_preproc);
	dynamic_reconfigure::Server<calib::PreprocessingConfig>::CallbackType f_preproc;
	f_preproc = boost::bind(&Preprocessing::conf_callback, &preproc, _1, _2);
	server_preproc.setCallback(f_preproc);

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