#include <registration/preprocessing_node.h>

using namespace std;

/*
*	args parser
*		node freq
*		filtering params
*		input topics namespace
*		input topics name
*		output topics namespace
*		registration algorithms
*/

const string inputs[] = {"/cam1", "/cam2", "/cam3"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string pc_topic_name = "/qhd/points";
const string pub_topic_name = "/reconstruction/point_clouds";
float frequency = 0;
vector<geometry::PointCloud*> PC;

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

/**
 * @brief Callback for subsampling dynamic reconfigure.
 */
void subsampling_conf_callback(registration::SubSamplingConfig &config, uint32_t level)
{
    bool enable = config.enable;

	/* if the subsampling size should be the same for every axis */
    float x = config.size / 1000;
    float y = config.size / 1000;
    float z = config.size / 1000;
    /*************************************************************/
	// this change should be made in the cfg file too
	/* if the subsampling size of each axis should be independant *
    float x = config.size_x;
    float y = config.size_y;
    float z = config.size_z;
    /**************************************************************/

	for (int i = 0 ; i < n_inputs ; i++)
	{
		subsampling_params_t subsampling_params = {enable, x, y, z};
		PC[i]->set_subsampling_params(subsampling_params);
	}

    ROS_DEBUG("Subsampling config updated");
}

/**
 * @brief Callback for cutting dynamic reconfigure.
 */
void cutting_conf_callback(registration::CuttingConfig &config, uint32_t level)
{
    bool x_enable = config.x_enable;
	float x_min = config.x_min / 1000;
	float x_max = config.x_max / 1000;
	bool y_enable = config.y_enable;
	float y_min = config.y_min / 1000;
	float y_max = config.y_max / 1000;
	bool z_enable = config.z_enable;
	float z_min = config.z_min / 1000;
	float z_max = config.z_max / 1000;

	for (int i = 0 ; i < n_inputs ; i++)
	{
		cutting_params_t cutting_params = {x_enable, x_min, x_max, y_enable, y_min, y_max, z_enable, z_min, z_max};
		PC[i]->set_cutting_params(cutting_params);
	}

    ROS_DEBUG("Cutting config updated");
}

/**
 * @brief Callback for radius filtering dynamic reconfigure.
 */
void radius_filtering_conf_callback(registration::RadiusFilteringConfig &config, uint32_t level)
{
    bool enable = config.enable;
	double radius = config.radius / 1000;
	int min_neighbors = config.min_neighbors;

	for (int i = 0 ; i < n_inputs ; i++)
	{
		radius_filtering_params_t radius_filtering_params = {enable, radius, min_neighbors};
		PC[i]->set_radius_filtering_params(radius_filtering_params);
	}

    ROS_DEBUG("Radius filtering config updated");
}

int main(int argc, char *argv[])
{
	// verbosity: debug
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "preprocessing_node");
		ros::NodeHandle nh;
		ros::NodeHandle nh_subsampling("~/subsampling");
		ros::NodeHandle nh_cutting("~/cutting");
		ros::NodeHandle nh_radius_filtering("~/radius_filtering");

	// create PointCloud objects
		for (int i = 0 ; i < n_inputs ; i++)
		{
			PC.push_back(new geometry::PointCloud(nh, get_topic_name(i), get_publish_name(i)));
		}

	// dynamic reconfigure
		dynamic_reconfigure::Server<registration::SubSamplingConfig> subsampling_srv(nh_subsampling);
		dynamic_reconfigure::Server<registration::CuttingConfig> cutting_srv(nh_cutting);
		dynamic_reconfigure::Server<registration::RadiusFilteringConfig> radius_filtering_srv(nh_radius_filtering);
		
		dynamic_reconfigure::Server<registration::SubSamplingConfig>::CallbackType subsampling_cb;
		dynamic_reconfigure::Server<registration::CuttingConfig>::CallbackType cutting_cb;
		dynamic_reconfigure::Server<registration::RadiusFilteringConfig>::CallbackType radius_filtering_cb;
		
		subsampling_cb = boost::bind(&subsampling_conf_callback, _1, _2);
		cutting_cb = boost::bind(&cutting_conf_callback, _1, _2);
		radius_filtering_cb = boost::bind(&radius_filtering_conf_callback, _1, _2);
		
		subsampling_srv.setCallback(subsampling_cb);
		cutting_srv.setCallback(cutting_cb);
		radius_filtering_srv.setCallback(radius_filtering_cb);

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