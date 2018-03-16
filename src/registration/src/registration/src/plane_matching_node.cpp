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
const string pub_topic_name = "/reconstruction/point_clouds";
float frequency = 2;
vector <geometry::Plane*> temp_planes;
vector <vector <geometry::Plane*> > planes;
tf::TransformListener *tf_listener;

vector <geometry::PointCloud*> PC;
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
 * @brief Returns the publishing opint cloud topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_pc_topic_name(int input_number)
{
	return pub_topic_name + inputs[input_number];
}

/**
 * @brief Returns the name of the topic to publish planes in.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_publish_name(int input_number)
{
  return pub_topic_name + "registered/" + inputs[input_number];
}

bool tf_exists(tf::TransformListener* tf_listener, tf::StampedTransform* tf, string name)
{
	bool exists = false;
	ros::Duration(0.1).sleep();
	try
	{
		tf_listener->lookupTransform("cam_center", name, ros::Time(0), *tf);
		exists = true;
	}
	catch (tf::TransformException &ex)
	{
		// ROS_DEBUG_STREAM(name << " not found");
	}
	return exists;
}

motion_t motion_from_plane_planes(const vector <geometry::Plane*> &sourcePlanes, const vector <geometry::Plane*> &targetPlanes)
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
	VectorXd ds(n_planes_1);
	VectorXd dt(n_planes_1);
	VectorXd d(n_planes_1);
	for (int line = 0; line < n_planes_1; line++)
	{
		tf::Vector3 normal_1 = sourcePlanes[line]->normal;
		tf::Vector3 normal_2 = targetPlanes[line]->normal;
		Ns(line, 0) = normal_1.getX();
		Ns(line, 1) = normal_1.getY();
		Ns(line, 2) = normal_1.getZ();
		Nt(line, 0) = normal_2.getX();
		Nt(line, 1) = normal_2.getY();
		Nt(line, 2) = normal_2.getZ();
		ds(line) = sourcePlanes[line]->d;
		dt(line) = targetPlanes[line]->d;
	}
	d = dt - ds;
	MatrixXd W = MatrixXd::Identity(n_planes_1, n_planes_1);
	Matrix3d Rhat = (Nt.transpose() * W * Nt).inverse() * (Nt.transpose() * W * Ns);
	Vector3d T = (Nt.transpose() * W * Nt).inverse() * (Nt.transpose() * W * d);

	JacobiSVD<Matrix3d> svd(Rhat, ComputeFullV | ComputeFullU);
	Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

	// compute residuals
		Matrix4d H = Matrix4d::Identity();
		H.topLeftCorner(3, 3) = R;
		H.topRightCorner(3, 1) = T;
		MatrixX4d planes_target(n_planes_1, 4);
		MatrixX4d planes_source(n_planes_1, 4);
		planes_source.topLeftCorner(n_planes_1, 3) = Ns;
		planes_source.topRightCorner(n_planes_1, 1) = ds;
		planes_target.topLeftCorner(n_planes_1, 3) = Nt;
		planes_target.topRightCorner(n_planes_1, 1) = dt;
		MatrixX4d residuals_mat;
		residuals_mat = planes_target - planes_source * H.inverse();

		vector <float> residuals_angle;
		vector <float> residuals_translation;
		for (int i = 0; i < n_planes_1; i++)
		{
			residuals_angle.push_back(sqrt(pow(residuals_mat(i, 0), 2.0) + pow(residuals_mat(i, 1), 2.0) + pow(residuals_mat(i, 2), 2.0)));
			residuals_translation.push_back(residuals_mat(i, 3));
		}

	Affine3d transform;
	transform.matrix() = H;
	tf::transformEigenToTF(transform, motion.H);
	motion.residuals_angle = residuals_angle;
	motion.residuals_translation = residuals_translation;

	return motion;
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

void pc_callback(sensor_msgs::PointCloud2 &pc, int i, ros::Publisher pc_pub)
{
	std::string frame = "registration";
	if (i == 0) frame = "cam_center";
	sensor_msgs::PointCloud2 temp;
	pcl_ros::transformPointCloud(frame, pc, temp, *tf_listener);
	pc_pub.publish(temp);
}

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
		static tf::TransformBroadcaster br;
    	tf::StampedTransform tf; 
		tf_listener = new tf::TransformListener();
		ros::Duration(0.5).sleep();
		// vector <ros::Subscriber> pc_sub;
		// vector <ros::Publisher> pc_pub;

		// for (int i = 0; i < n_inputs; i++)
		// {
		// 	ros::Subscriber temp_sub;
		// 	ros::Publisher temp_pub;
		// 	temp_sub = nh.subscribe(get_pc_topic_name(i), 1, boost::bind(pc_callback, _1, i));
    	// 	temp_pub = nh.advertise<sensor_msgs::PointCloud2>(get_publish_name(i), 1);
		// 	pc_sub.push_back(temp_sub);
		// 	pc_pub.push_back(temp_pub);
		// }

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
				while (tf_exists(tf_listener, &tf, get_topic_name(i, n+1)))
				{
					temp_planes.push_back(new geometry::Plane(tf));
					n++;
				}
				planes.push_back(temp_planes);
				ROS_DEBUG_STREAM(patch::to_string(n) << " planes detected for " << inputs[i]);
		}

		motion_t motion = motion_from_plane_planes(planes[0], planes[1]);
		br.sendTransform(tf::StampedTransform(motion.H, ros::Time::now(), "cam_center", "registration"));

		ROS_DEBUG_STREAM("Mean rotation error: " << accumulate(motion.residuals_angle.begin(), motion.residuals_angle.end(), 0.0) / motion.residuals_angle.size() * 100 << "%");
		ROS_DEBUG_STREAM("Mean translation error: " << accumulate(motion.residuals_translation.begin(), motion.residuals_translation.end(), 0.0) / motion.residuals_translation.size() * 1000 << "mm");

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