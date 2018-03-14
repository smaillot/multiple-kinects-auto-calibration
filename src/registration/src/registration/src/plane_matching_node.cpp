#include <registration/plane_matching_node.h>

using namespace std;
using namespace Eigen;
/*
*	args parser
*
*	node frequency
*   input list
*	input topics namespace
*	output topics namespace
*/

const string inputs[2] = {"/cam1", "/cam2"};
const int n_inputs = sizeof(inputs) / sizeof(*inputs);
const string sub_topic_name = "/reconstruction/planes";
const string pub_topic_name = "/reconstruction/lines";
float frequency = 2;
vector <geometry::Plane*> temp_planes;
vector <vector <geometry::Plane*> > planes;
// vector <vector <geometry::PointCloud*> > PC[n_inputs];

/**
 * @brief Returns the publishing topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_topic_name(int input_number, int plane)
{
	return sub_topic_name + inputs[input_number] + "/plane" + patch::to_string(plane);
}

/**
 * @brief Returns the name of the topic to publish planes in.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_publish_name(int input_number, int plane_number)
{
  return pub_topic_name + inputs[input_number] + "_" + patch::to_string(plane_number) + "_registered";
}

bool tf_exists(tf::TransformListener* tf_listener, tf::StampedTransform* tf, string name)
{
	bool exists = false;
	ros::Duration(0.1).sleep();
	try
	{
		tf_listener->lookupTransform(REFERENCE_FRAME, name, ros::Time(0), *tf);
		exists = true;
	}
	catch (tf::TransformException &ex)
	{
		// ROS_DEBUG_STREAM(name << " not found");
	}
	return exists;
}

motion_t motion_from_plane_planes(vector <geometry::Plane*> &sourcePlanes, vector <geometry::Plane*> &targetPlanes)
{
	int n_planes_1 = sourcePlanes.size();
	int n_planes_2 = targetPlanes.size();
	motion_t motion;

	if (n_planes_1 != n_planes_2)
	{
		ROS_ERROR("The number of planes detected in the 2 sets must be equal");
		return motion;
	}
	if (n_planes_1 < 3)
	{
		ROS_ERROR("At least 3 planes are needed");
		return motion;
	}

	MatrixX3d Ns(n_planes_1, 3);
	MatrixX3d Nt(n_planes_1, 3);
	VectorXd d(n_planes_1);
	for (int line = 0; line < n_planes_1; line++)
	{
		tf::Vector3 normal_1 = sourcePlanes[line]->normal;
		tf::Vector3 normal_2 = targetPlanes[line]->normal;
		Ns(line, 1) = normal_1.getX();
		Ns(line, 2) = normal_1.getY();
		Ns(line, 3) = normal_1.getZ();
		Nt(line, 1) = normal_2.getX();
		Nt(line, 2) = normal_2.getY();
		Nt(line, 3) = normal_2.getZ();
		d(line) = sourcePlanes[line]->d - targetPlanes[line]->d;
	}
	MatrixXd W = MatrixXd::Identity(n_planes_1, n_planes_1);
	Matrix3d Rhat = (Nt.transpose() * W * Nt).inverse() * (Nt.transpose() * W * Ns);
	Vector3d T = (Nt.transpose() * W * Nt).inverse() * (Nt.transpose() * W * d);

	JacobiSVD<Matrix3d> svd(Rhat, ComputeThinU | ComputeThinV);
	Matrix3d = svd.matrixU() * svd.matrixV().transpose();

	// compute residual
}

// /**
//  * @brief Callback for plane matching parameters dynamic reconfigure reconfigure.
//  */
// void plane_matching_conf_callback(plane_matching::PlaneMatchingConfig &config, uint32_t level)
// {
//     bool enabled = config.enabled;
// 	int n_planes = config.n_planes;
// 	float th_dist = config.th_dist / 1000;
// 	int max_it = config.max_it;

// 	for (int i = 0 ; i < n_inputs ; i++)
// 	{
// 		PD[i]->set_params(enabled, n_planes, th_dist, max_it);
// 	}

//     ROS_DEBUG("Plane detection parameters config updated");
// }

int main(int argc, char *argv[])
{
	// verbosity: debug
		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}

	// Initialize ROS
		ros::init(argc, argv, "plane_matching_node");
		ros::NodeHandle nh;
		ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(pub_topic_name, 100);
		// ros::NodeHandle node_ransac("~/RANSAC");
    	tf::TransformListener tf_listener; 
		static tf::TransformBroadcaster br;
    	tf::StampedTransform tf; 
		ros::Duration(0.5).sleep(); // for for tf listener initialization
		bool new_planes;

	// // dynamic reconfigure
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig> plane_detection_srv(node_ransac);
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig>::CallbackType plane_detection_cb;
	// 	plane_detection_cb = boost::bind(&plane_detection_conf_callback, _1, _2);
	// 	plane_detection_srv.setCallback(plane_detection_cb);

	// ROS loop
	ros::Rate loop_rate(frequency);
	while (ros::ok())
	{
		planes.clear();
		for (int i = 0; i < n_inputs; i++)
		{
			// update planes
				temp_planes.clear();
				int n = 0;
				while (tf_exists(&tf_listener, &tf, get_topic_name(i, n+1)))
				{
					temp_planes.push_back(new geometry::Plane(tf));
					n++;
				}
				planes.push_back(temp_planes);
			// // update point clouds if needed
			// 	if (new_planes) 
			// 	{
			// 		PC[i].clear();
			// 		ROS_DEBUG_STREAM(patch::to_string(n) << " planes detected for " << inputs[i]);
			// 		for (int j = 0 ; j < n ; j++)
			// 		{
			// 			PC[i].push_back(new geometry::PointCloud(nh, get_topic_name(i, j), get_publish_name(i, j)));
			// 		}
			// 	}
		}

	// sleep
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