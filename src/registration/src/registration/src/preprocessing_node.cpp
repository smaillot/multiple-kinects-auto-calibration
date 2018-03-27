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

vector <string> inputs; // = ("/cam1/qhd/points", "/cam2/qhd/points", "/cam3/qhd/points");
vector <string> input_names; // = ("cam1", "cam2", "cam3");
int n_inputs; // = sizeof(inputs) / sizeof(*inputs);
const string pub_topic_name = "/reconstruction/point_clouds";
float frequency = 0;
vector<geometry::PointCloud*> PC;

std::string get_topic_name(int input_number)
{
	return inputs[input_number];
}

std::string get_publish_name(int input_number)
{
	return pub_topic_name + "/" + input_names[input_number];
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
void subsampling_conf_callback(registration::SubSamplingConfig &config, uint32_t level, int i)
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

	subsampling_params_t subsampling_params = {enable, x, y, z};
	PC[i]->set_subsampling_params(subsampling_params);

    ROS_DEBUG_STREAM("Subsampling config updated for " + input_names[i]);
}

/**
 * @brief Callback for cutting dynamic reconfigure.
 */
void cutting_conf_callback(registration::CuttingConfig &config, uint32_t level, int i)
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
	
	cutting_params_t cutting_params = {x_enable, x_min, x_max, y_enable, y_min, y_max, z_enable, z_min, z_max};
	PC[i]->set_cutting_params(cutting_params);

    ROS_DEBUG_STREAM("Cutting config updated for " + input_names[i]);
}

/**
 * @brief Callback for radius filtering dynamic reconfigure.
 */
void radius_filtering_conf_callback(registration::RadiusFilteringConfig &config, uint32_t level, int i)
{
    bool enable = config.enable;
	double radius = config.radius / 1000;
	int min_neighbors = config.min_neighbors;

	radius_filtering_params_t radius_filtering_params = {enable, radius, min_neighbors};
	PC[i]->set_radius_filtering_params(radius_filtering_params);

    ROS_DEBUG_STREAM("Radius filtering config updated for " + input_names[i]);
}

int main(int argc, char *argv[])
{
	// verbosity: debug
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// parsing arguments

		// TODO error catching 

		inputs.clear();
		input_names.clear();
		int i = 0;
		while (i < argc-1) 
		{
			input_names.push_back(argv[i+1]);
			i++;
			inputs.push_back(argv[i+1]);
			i++;
		}
		n_inputs = inputs.size();

	// Initialize ROS
		string node_name = "preprocessing";
		for (int i = 0; i < n_inputs; i++)
		{
			node_name += "_" + input_names[i];
		}
		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;
		vector <ros::NodeHandle> nh_subsampling;
		vector <ros::NodeHandle> nh_cutting;
		vector <ros::NodeHandle> nh_radius_filtering;


	// create PointCloud objects
		for (int i = 0 ; i < n_inputs ; i++)
		{
			PC.push_back(new geometry::PointCloud(nh, get_topic_name(i), get_publish_name(i)));

			nh_subsampling.push_back(ros::NodeHandle("preprocessing/" + input_names[i] + "/subsampling"));
			nh_cutting.push_back(ros::NodeHandle("preprocessing/" + input_names[i] + "/cutting"));
			nh_radius_filtering.push_back(ros::NodeHandle("preprocessing/" + input_names[i] + "/radius_filtering"));
		}

	// dynamic reconfigure
	vector <dynamic_reconfigure::Server<registration::SubSamplingConfig>* > subsampling_srv;
	vector <dynamic_reconfigure::Server<registration::CuttingConfig>* > cutting_srv;
	vector <dynamic_reconfigure::Server<registration::RadiusFilteringConfig>* > radius_filtering_srv;
	dynamic_reconfigure::Server<registration::SubSamplingConfig>::CallbackType subsampling_cb[n_inputs];
	dynamic_reconfigure::Server<registration::CuttingConfig>::CallbackType cutting_cb[n_inputs];
	dynamic_reconfigure::Server<registration::RadiusFilteringConfig>::CallbackType radius_filtering_cb[n_inputs];
	for (int i = 0 ; i < n_inputs ; i++)
	{		
		subsampling_srv.push_back(new dynamic_reconfigure::Server<registration::SubSamplingConfig>(nh_subsampling[i]));
		cutting_srv.push_back(new dynamic_reconfigure::Server<registration::CuttingConfig>(nh_cutting[i]));
		radius_filtering_srv.push_back(new dynamic_reconfigure::Server<registration::RadiusFilteringConfig>(nh_radius_filtering[i]));

		subsampling_cb[i] = boost::bind(&subsampling_conf_callback, _1, _2, i);
		cutting_cb[i] = boost::bind(&cutting_conf_callback, _1, _2, i);
		radius_filtering_cb[i] = boost::bind(&radius_filtering_conf_callback, _1, _2, i);
		
		subsampling_srv[i]->setCallback(subsampling_cb[i]);
		cutting_srv[i]->setCallback(cutting_cb[i]);
		radius_filtering_srv[i]->setCallback(radius_filtering_cb[i]);
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