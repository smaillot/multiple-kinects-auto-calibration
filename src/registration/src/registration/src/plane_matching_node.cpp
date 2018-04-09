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

/* metaparameters influencing registration precision

* number of planes
* weight (trust) for each plane in W matrix
* algorithm (plane-plane, points-plane)
* plane matchings

*/

vector <string> inputs;
int n_inputs;
const string sub_topic_name = "/reconstruction/planes";
const string pub_topic_name = "/reconstruction/point_clouds";
float frequency = 0;
vector <geometry::Plane*> temp_planes;
vector <vector <geometry::Plane*> > planes;
PCRegistered* PC;

/**
 * @brief Returns the publishing topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_topic_name(int input_number, int plane)
{
	return sub_topic_name + "/" + inputs[input_number] + "/plane" + patch::to_string(plane);
}

/**
 * @brief Returns the publishing opint cloud topic of a given camera.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_pc_topic_name(int input_number)
{
	return pub_topic_name + "/" + inputs[input_number]; // + "/preprocessed";
}

/**
 * @brief Returns the name of the topic to publish planes in.
 *
 * @params input_number Camera ID in the input list.
 */
std::string get_publish_name(int input_number)
{
  return pub_topic_name + "/" + inputs[input_number] + "/registered";
}

bool tf_exists(tf::TransformListener* tf_listener, tf::StampedTransform* transf, string name)
{
	bool exists = false;
	ros::Duration(0.1).sleep();
	try
	{
		tf_listener->lookupTransform("cam_center", name, ros::Time(0), *transf);
		exists = true;
	}
	catch (tf::TransformException &ex)
	{
		ROS_DEBUG_STREAM(name << " not found");
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
	// W(2,2) = 0.001; // the 3rd plane is less to be trusted, we need it only to fix y translation
	
	ROS_DEBUG_STREAM("Ns = \n" << Ns);
	ROS_DEBUG_STREAM("Nt = \n" << Nt);
	ROS_DEBUG_STREAM("d = \n" << d);
	ROS_DEBUG_STREAM("W = \n" << W);

	Matrix3d Rhat = (Nt.transpose() * W * Nt).inverse() * (Nt.transpose() * W * Ns);
	Vector3d T = -(Nt.transpose() * W * Nt).inverse() * (Nt.transpose() * W * d);

	ROS_DEBUG_STREAM("rotation estimate = \n" << Rhat);
	ROS_DEBUG_STREAM("translation = \n" << T);

	JacobiSVD<Matrix3d> svd(Rhat, ComputeFullV | ComputeFullU);
	Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

	ROS_DEBUG_STREAM("rotation normalized = \n" << R);

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
		ROS_DEBUG("\nresiduals:");
		for (int i = 0; i < n_planes_1; i++)
		{
			float angle = sqrt(pow(residuals_mat(i, 0), 2.0) + pow(residuals_mat(i, 1), 2.0) + pow(residuals_mat(i, 2), 2.0));
			float transl = residuals_mat(i, 3);
			residuals_angle.push_back(angle);
			residuals_translation.push_back(transl);
			ROS_DEBUG_STREAM("\tplane " << i);
			ROS_DEBUG_STREAM("\t\ttranslation: " << transl);
			ROS_DEBUG_STREAM("\t\tangle: " << angle);
		}


	Affine3d transform;
	transform.matrix() = H;
	tf::transformEigenToTF(transform, motion.H);
	motion.mat = H;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr create_pc(vector <geometry::Plane*> planes)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width	= planes.size();
	cloud->height	= 1;
	cloud->is_dense	= false;
	cloud->points.resize(cloud->width * cloud->height);
	for (int i = 0; i < planes.size(); i++)
	{
		cloud->points[i].x	= planes[i]->normal.getX();
		cloud->points[i].y	= planes[i]->normal.getY();
		cloud->points[i].z	= planes[i]->normal.getZ();
	}
	return cloud;
}

double point_dist(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
	double d = 0;
	d += pow(p1.x - p2.x, 2);
	d += pow(p1.y - p2.y, 2);
	d += pow(p1.z - p2.z, 2);

	return sqrt(d);
}

MatrixXd compute_dist(pcl::PointCloud<pcl::PointXYZ>::Ptr pc1, pcl::PointCloud<pcl::PointXYZ>::Ptr pc2)
{
	MatrixXd mat(planes[0].size(), planes[1].size());
	for (int i = 0; i < planes[0].size(); i++)
	{
		for (int j = 0; j < planes[1].size(); j++)
		{
			mat(i, j) = point_dist(pc1->points[i], pc2->points[j]);
		}
	}
	return mat;
}

void debug_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, string name)
{
	ROS_DEBUG_STREAM(name);
	for (int i = 0; i < pc->points.size(); i++)
	{
		ROS_DEBUG_STREAM(patch::to_string(pc->points[i].x) << " " << patch::to_string(pc->points[i].y) << " " << patch::to_string(pc->points[i].z));
	}
}

vector <vector <geometry::Plane*> > match_planes(vector <vector <geometry::Plane*> > planes, double th)
{
  	pcl::PointCloud<pcl::PointXYZ>::Ptr normals1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normals2(new pcl::PointCloud<pcl::PointXYZ>);
	normals1 = create_pc(planes[0]);
	normals2 = create_pc(planes[1]);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(normals1);
	icp.setInputTarget(normals2);

  	pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*res);

	debug_pc(normals1, "planes1");
	debug_pc(normals2, "planes2");
	debug_pc(res, "planes1 after transform");

	MatrixXd dist_mat = compute_dist(res, normals2);

	vector <vector <int> > matches; 
	for (int i = 0; i < planes[0].size(); i++)
	{
		for (int j = 0; j < planes[1].size(); j++)
		{
			double dist = dist_mat(i, j);
			ROS_DEBUG_STREAM("Trying to match " + inputs[0] + "/plane" + patch::to_string(i) + " with " + inputs[1] + "/plane" + patch::to_string(j) + ": normal dist = " + patch::to_string(dist) + " / " + patch::to_string(th));
			if (dist < th)
			{
				vector <int> match(2, 0);
				match[0] = i;
				match[1] = j;
				matches.push_back(match);
			}
		}
	}

	vector <vector <geometry::Plane*> > output;
	vector <geometry::Plane*> planes1;
	vector <geometry::Plane*> planes2;

	for (int i = 0; i < matches.size(); i++)
	{
		planes1.push_back(planes[0][matches[i][0]]);
		planes2.push_back(planes[1][matches[i][1]]);
		ROS_DEBUG_STREAM(inputs[0] + "/plane" + patch::to_string(matches[i][0]) + " matched with " + inputs[1] + "/plane" + patch::to_string(matches[i][1]));
	}

	output.push_back(planes1);
	output.push_back(planes2);

	return output;
}

void PCRegistered::pc_callback(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    tf::StampedTransform transf; 
	planes.clear();
	for (int i = 0; i < 2; i++)
	{
		// update planes
			temp_planes.clear();
			int n = 0;
			while (tf_exists(&this->tf_listener, &transf, get_topic_name(i, n+1)))
			{
				temp_planes.push_back(new geometry::Plane(transf));
				n++;
			}
			planes.push_back(temp_planes);
			ROS_INFO_STREAM(patch::to_string(n) << " planes detected for " << inputs[i]);
	}

	vector <vector <geometry::Plane*> > matched_planes = match_planes(planes, 0.2);
	motion_t motion = motion_from_plane_planes(matched_planes[0], matched_planes[1]);
	string new_frame = "registration";
	for (int i=0; i < 2; i++)
	{
		new_frame += "_" + inputs[i];
	}
	this->br.sendTransform(tf::StampedTransform(motion.H, ros::Time::now(), "cam_center", "/" + new_frame));

	ROS_DEBUG_STREAM("Mean rotation error: " << accumulate(motion.residuals_angle.begin(), motion.residuals_angle.end(), 0.0) / motion.residuals_angle.size() * 100 << "%");
	ROS_DEBUG_STREAM("Mean translation error: " << accumulate(motion.residuals_translation.begin(), motion.residuals_translation.end(), 0.0) / motion.residuals_translation.size() * 1000 << "mm");

	sensor_msgs::PointCloud2 input = *pc;
	sensor_msgs::PointCloud2 output = input;
	pcl_ros::transformPointCloud(motion.mat.cast <float>(), input, output);
	// if (this->tf_listener.waitForTransform(pc->header.frame_id, new_frame, ros::Time(0), ros::Duration(1.0)))
	// {
	// 	pcl_ros::transformPointCloud(new_frame, input, output, this->tf_listener);
	// }
	// else
	// {
    //     ROS_ERROR_STREAM("No tf received from " + pc->header.frame_id + " to " + new_frame + " in 1s. Abort point cloud transform.");
	// 	output = input;
	// }
	// if (output.data.size() == 0)
	// {
	// 	ROS_WARN("Registered point cloud is empty");
	// 	output = input;
	// }

	this->pc_pub.publish(output);
	ROS_DEBUG_STREAM("Publish registered point cloud on " + this->pub_name + " (" + patch::to_string(output.data.size()) + " points)");
}

PCRegistered::PCRegistered(std::string sub_name, std::string pub_name, ros::NodeHandle node)
{
	this->sub_name = sub_name;
	this->pub_name = pub_name;
	this->node = node;
	this->pc_sub = this->node.subscribe(this->sub_name, 1, &PCRegistered::pc_callback, this);
	this->pc_pub = this->node.advertise<sensor_msgs::PointCloud2>(this->pub_name, 1);
}

int main(int argc, char *argv[])
{
	// TODO error catching 
		inputs.clear();
		inputs.push_back(argv[1]);
		inputs.push_back(argv[2]);
		n_inputs = inputs.size();

	// Initialize ROS
		string node_name = "plane_matching_" + inputs[0] + "_to_" + inputs[1];
		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;
		ros::Duration(3).sleep();

		PC = new PCRegistered(get_pc_topic_name(0), get_publish_name(0), nh);
		
	// // dynamic reconfigure
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig> plane_detection_srv(node_ransac);
	// 	dynamic_reconfigure::Server<plane_detection::PlaneDetectionConfig>::CallbackType plane_detection_cb;
	// 	plane_detection_cb = boost::bind(&plane_detection_conf_callback, _1, _2);
	// 	plane_detection_srv.setCallback(plane_detection_cb);

	// ROS loop
	ros::Rate loop_rate(frequency);
	while (ros::ok())
	{
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
	}/*

	geometry::Plane* p1 = new geometry::Plane(tf::Vector3(1, 0, 0), tf::Vector3(0, 0, 0));
	geometry::Plane* p2 = new geometry::Plane(tf::Vector3(0, 1, 0), tf::Vector3(0, 0, 0));
	geometry::Plane* p3 = new geometry::Plane(tf::Vector3(0, 0, 1), tf::Vector3(0, 0, 0));
	ROS_DEBUG_STREAM("\n" << *p1 << "\n" << *p2 << "\n" << *p3);

	geometry::Plane* q1 = new geometry::Plane(tf::Vector3(0, 1, 0), tf::Vector3(1, 0, 0));
	geometry::Plane* q2 = new geometry::Plane(tf::Vector3(-1, 0, 0), tf::Vector3(1, 0, 0));
	geometry::Plane* q3 = new geometry::Plane(tf::Vector3(0, 0, 1), tf::Vector3(1, 0, 0));
	ROS_DEBUG_STREAM("\n" << *q1 << "\n" << *q2 << "\n" << *q3);

	temp_planes.push_back(p1);
	temp_planes.push_back(p2);
	temp_planes.push_back(p3);
	planes.push_back(temp_planes);

	temp_planes.clear();
	temp_planes.push_back(q1);
	temp_planes.push_back(q2);
	temp_planes.push_back(q3);
	planes.push_back(temp_planes);

	motion_t mot = motion_from_plane_planes(planes[0], planes[1]);*/

	return 0;
}