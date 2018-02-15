#include "ros/ros.h"
// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <pc_processing/subsamplingConfig.h>
// My libraries
#include <pc_processing/PcProcessing.h>

double subsize;

void subsampling_callback(pc_processing::subsamplingConfig &config, uint32_t level)
{
    subsize = config.subsampling_enable * config.subsampling_size / 100;
}

// void send_parameter(ros::NodeHandle* nh, std::string name, double val)
// {
// 	nh->setParam(name, val);
// }

int main(int argc, char *argv[])
    // Initialize ROS
{
    ros::init(argc, argv, "registration_params");
    ros::NodeHandle nh;

    // dynamic reconfigure
    dynamic_reconfigure::Server<pc_processing::subsamplingConfig> server;
    dynamic_reconfigure::Server<pc_processing::subsamplingConfig>::CallbackType subsampling_cb;
    subsampling_cb = boost::bind(&subsampling_callback, _1, _2);
    server.setCallback(subsampling_cb);

	// send_parameter(&nh, "/scene/registration/subsampling/subsampling_size", subsize);

	ros::spin();
}