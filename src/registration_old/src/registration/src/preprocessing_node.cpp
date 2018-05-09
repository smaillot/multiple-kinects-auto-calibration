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

string topic; // = ("/cam1/qhd/points", "/cam2/qhd/points", "/cam3/qhd/points");
string input_name; // = ("cam1", "cam2", "cam3");
const string pub_topic_name = "/reconstruction/point_clouds";
float frequency = 0;
geometry::PointCloud* PC;
ros::MultiThreadedSpinner spinner(frequency);

std::string get_publish_name()
{
	return pub_topic_name + "/" + input_name;
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

	subsampling_params_t subsampling_params = {enable, x, y, z};
	PC->set_subsampling_params(subsampling_params);

    ROS_DEBUG_STREAM("Subsampling config updated for " + input_name);
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
	
	cutting_params_t cutting_params = {x_enable, x_min, x_max, y_enable, y_min, y_max, z_enable, z_min, z_max};
	PC->set_cutting_params(cutting_params);

    ROS_DEBUG_STREAM("Cutting config updated for " + input_name);
}

/**
 * @brief Callback for radius filtering dynamic reconfigure.
 */
void radius_filtering_conf_callback(registration::RadiusFilteringConfig &config, uint32_t level)
{
    bool enable = config.enable;
	double radius = config.radius / 1000;
	int min_neighbors = config.min_neighbors;

	radius_filtering_params_t radius_filtering_params = {enable, radius, min_neighbors};
	PC->set_radius_filtering_params(radius_filtering_params);

    ROS_DEBUG_STREAM("Radius filtering config updated for " + input_name);
}

void outliers_removal_conf_callback(registration::OutliersRemovalConfig &config, uint32_t level)
{
    bool enable = config.enable;
	int meank = config.meank;
	float std_mul = config.std_mul;

	outliers_removal_params_t outliers_removal_params = {enable, meank, std_mul};
	PC->set_outliers_removal_params(outliers_removal_params);

    ROS_DEBUG_STREAM("Outliers removal config updated for " + input_name);
}

void kp_conf_callback(registration::KeypointsConfig &config, uint32_t level)
{
    int method = config.method;
	float support_radius = config.support_radius / 1000;
	float nms_radius = config.nms_radius / 1000;
	float min_scale = config.min_scale / 1000;
	int nr_octave = config.nr_octave;
	int nr_scales_per_oct = config.nr_scales_per_oct;
	float min_contrast = config.min_contrast;
	float nrad = config.nrad / 1000;

	kp_params_t kp_params = {method, support_radius, nms_radius, min_scale, nr_octave, nr_scales_per_oct, min_contrast, nrad};
	PC->set_kp_params(kp_params);

    ROS_DEBUG_STREAM("Outliers removal config updated for " + input_name);
}

int main(int argc, char *argv[])
{
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
        {
            ros::console::notifyLoggerLevelsChanged();
        }

	// parsing arguments

		// TODO error catching 

		input_name = argv[1];
		topic = argv[2];
		string f(argv[3]);
		frequency = (float)atof(f.c_str());

	// Initialize ROS
		string node_name = "preprocessing_";
		node_name += input_name;
		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;

	ros::NodeHandle nh_subsampling("preprocessing/" + input_name + "/subsampling");
	PC = new geometry::PointCloud(nh, topic, get_publish_name());
	PC->change_frame("cam_center");
	ros::NodeHandle nh_cutting("preprocessing/" + input_name + "/cutting");
	ros::NodeHandle nh_radius_filtering("preprocessing/" + input_name + "/radius_filtering");
	ros::NodeHandle nh_outliers_removal("preprocessing/" + input_name + "/outliers_removal");
	ros::NodeHandle nh_kp("preprocessing/" + input_name + "/keypoints_extraction");

	// dynamic reconfigure
			
		dynamic_reconfigure::Server<registration::SubSamplingConfig> subsampling_srv(nh_subsampling);
		dynamic_reconfigure::Server<registration::CuttingConfig> cutting_srv(nh_cutting);
		dynamic_reconfigure::Server<registration::RadiusFilteringConfig> radius_filtering_srv(nh_radius_filtering);
		dynamic_reconfigure::Server<registration::OutliersRemovalConfig> outliers_removal_srv(nh_outliers_removal);
		dynamic_reconfigure::Server<registration::KeypointsConfig> kp_srv(nh_kp);
	
		subsampling_srv.setCallback(boost::bind(&subsampling_conf_callback, _1, _2));
		cutting_srv.setCallback(boost::bind(&cutting_conf_callback, _1, _2));
		radius_filtering_srv.setCallback(boost::bind(&radius_filtering_conf_callback, _1, _2));
		outliers_removal_srv.setCallback(boost::bind(&outliers_removal_conf_callback, _1, _2));
		kp_srv.setCallback(boost::bind(&kp_conf_callback, _1, _2));

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