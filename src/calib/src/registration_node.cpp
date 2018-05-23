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

void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc1, const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    sensor_msgs::PointCloud2 input1 = *pc1;
    sensor_msgs::PointCloud2 input2 = *pc2;
    sensor_msgs::PointCloud2 output1 = *pc1;
    sensor_msgs::PointCloud2 output2 = *pc2;
    sensor_msgs::PointCloud2 merged_pc;
    pcl::concatenatePointCloud(output1, output2, merged_pc);

    pub.publish(merged_pc);
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
    ros::NodeHandle node_scene("/calib/scene");
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
	{
		ros::console::notifyLoggerLevelsChanged();
	}

	dynamic_reconfigure::Server<calib::RegistrationConfig> server(nh);
	dynamic_reconfigure::Server<calib::RegistrationConfig>::CallbackType f;
	f = boost::bind(&conf_callback, _1, _2);
	server.setCallback(f);

	pub = nh.advertise<sensor_msgs::PointCloud2>("/calib/clouds/scene", 1);

	message_filters::Subscriber<sensor_msgs::PointCloud2> cam1_sub(nh, "/calib/clouds/cam1/preproc", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cam2_sub(nh, "/calib/clouds/cam2/preproc", 1);
	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> KinectSync;
	Synchronizer<KinectSync> sync(KinectSync(10), cam1_sub, cam2_sub);
	sync.registerCallback(boost::bind(&pc_callback, _1, _2));

///////////
// main code

	Cloud cam1(&node_cam1, "/cam1/qhd/points", "/calib/clouds/cam1");
	dynamic_reconfigure::Server<calib::CloudConfig> server_cam1(node_cam1);
	dynamic_reconfigure::Server<calib::CloudConfig>::CallbackType f_cam1;
	f_cam1 = boost::bind(&Cloud::conf_callback, &cam1, _1, _2);
	server_cam1.setCallback(f_cam1);

	Cloud cam2(&node_cam2, "/cam2/qhd/points", "/calib/clouds/cam2");
	dynamic_reconfigure::Server<calib::CloudConfig> server_cam2(node_cam2);
	dynamic_reconfigure::Server<calib::CloudConfig>::CallbackType f_cam2;
	f_cam2 = boost::bind(&Cloud::conf_callback, &cam2, _1, _2);
	server_cam2.setCallback(f_cam2);

	Cloud scene(&node_cam2, "/calib/clouds/scene", "/calib/clouds/scene");
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