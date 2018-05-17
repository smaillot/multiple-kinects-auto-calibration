#include "registration_node.h"

using namespace std;
using namespace message_filters;

float freq = 0;
ros::Publisher pub;

void conf_callback(calib::RegistrationConfig &config, uint32_t level)
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

void pc_callback(const pcConstPtr& pc1, const pcConstPtr& pc2)
{
	pc_t input1 = *pc1;
	pc_t input2 = *pc2;
	pc_t merged = input1 + input2;
    pub.publish(merged);
}

int main(int argc, char *argv[])
{
///////////
// init

    string node_name = (string) "registration_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("/calib/registration");
    ros::NodeHandle node_cam1("/calib/cam1");
    ros::NodeHandle node_cam2("/calib/cam2");
    ros::NodeHandle node_match("/calib/match");
    ros::NodeHandle node_scene("/calib/scene");
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
	{
		ros::console::notifyLoggerLevelsChanged();
	}

	dynamic_reconfigure::Server<calib::RegistrationConfig> server(nh);
	dynamic_reconfigure::Server<calib::RegistrationConfig>::CallbackType f;
	f = boost::bind(&conf_callback, _1, _2);
	server.setCallback(f);

	pub = nh.advertise<pc_msg_t>("/calib/clouds/scene", 1);

	message_filters::Subscriber<pc_t> cam1_sub(nh, "/calib/clouds/cam1", 1);
	message_filters::Subscriber<pc_t> cam2_sub(nh, "/calib/clouds/cam2", 1);
	typedef sync_policies::ApproximateTime<pc_t, pc_t> KinectSync;
	Synchronizer<KinectSync> sync(KinectSync(10), cam1_sub, cam2_sub);
	sync.registerCallback(boost::bind(&pc_callback, _1, _2));

///////////
// main code

	Cloud cam1(&node_cam1, "/cam1/qhd/points", "cam1");
	dynamic_reconfigure::Server<calib::CloudConfig> server_cam1(node_cam1);
	dynamic_reconfigure::Server<calib::CloudConfig>::CallbackType f_cam1;
	f_cam1 = boost::bind(&Cloud::conf_callback, &cam1, _1, _2);
	server_cam1.setCallback(f_cam1);

	Cloud cam2(&node_cam2, "/cam2/qhd/points", "cam2");
	dynamic_reconfigure::Server<calib::CloudConfig> server_cam2(node_cam2);
	dynamic_reconfigure::Server<calib::CloudConfig>::CallbackType f_cam2;
	f_cam2 = boost::bind(&Cloud::conf_callback, &cam2, _1, _2);
	server_cam2.setCallback(f_cam2);

	Merging match(&node_match, "cam1", "cam2");
	dynamic_reconfigure::Server<calib::MergingConfig> server_merging(node_match);
	dynamic_reconfigure::Server<calib::MergingConfig>::CallbackType f_match;
	f_match = boost::bind(&Merging::conf_callback, &match, _1, _2);
	server_merging.setCallback(f_match);

	Cloud scene(&node_cam2, "/calib/clouds/scene", "scene");
	dynamic_reconfigure::Server<calib::CloudConfig> server_scene(node_scene);
	dynamic_reconfigure::Server<calib::CloudConfig>::CallbackType f_scene;
	f_scene = boost::bind(&Cloud::conf_callback, &scene, _1, _2);
	server_scene.setCallback(f_scene);

////////////
// ros loop

	while (ros::ok())
	{
		if (freq > 0)
		{
			ros::Rate loop_rate(freq);
			ros::spinOnce();
			loop_rate.sleep();
		}
		else
		{
			ros::spin();
		}
	}

	return 0;
///////////
}