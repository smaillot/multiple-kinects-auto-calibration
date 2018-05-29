#include "transform_estimation_node.h"
 
using namespace std;
 
float freq = 0;
TransformEstimator te;
string name;
float weight;
bool inverse;
string frame;

void callback(const calib::MatchesConstPtr& matches)
{
    te.reset();
    vector<Eigen::Vector4f> planes1;
    vector<Eigen::Vector4f> planes2;
    vector<Eigen::Vector3f> points1;
    vector<Eigen::Vector3f> points2;
    for (int i = 0; i < matches->planes1.size(); i++)
    {
      planes1.push_back(Eigen::Vector4f(matches->planes1[i].coef[0], matches->planes1[i].coef[1], matches->planes1[i].coef[2], matches->planes1[i].coef[3]));
      planes2.push_back(Eigen::Vector4f(matches->planes2[i].coef[0], matches->planes2[i].coef[1], matches->planes2[i].coef[2], matches->planes2[i].coef[3]));
    }
    pcPtr keypoints1(new pc_t);
    convert(matches->points1, keypoints1);
    pcPtr keypoints2(new pc_t);
    convert(matches->points2, keypoints2);

    for (int i = 0; i < keypoints1->points.size(); i++)
    {
      points1.push_back(Eigen::Vector3f(keypoints1->points[i].x, keypoints1->points[i].y, keypoints1->points[i].z));
      points2.push_back(Eigen::Vector3f(keypoints2->points[i].x, keypoints2->points[i].y, keypoints2->points[i].z));
    }
    // points1.push_back(Eigen::Vector3f(0,0,0));
    // points2.push_back(Eigen::Vector3f(0,0,0));
    te.addPlanes(planes1, true, weight);
    te.addPlanes(planes2, false, weight);
    te.addPoints(points1, true);
    te.addPoints(points2, false);
    Eigen::Affine3d T(te.getTransform());

    tf::Transform transform;
    if (!inverse)
    {
      tf::transformEigenToTF(T.inverse(), transform);
    }
    else
    {
      tf::transformEigenToTF(T, transform);
    }
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, name));
}

void conf_callback(calib::TransformEstimationConfig &config, uint32_t level)
{
  weight = config.planes_weight;
  inverse = config.inverse;
  frame = config.frame;
}

int main(int argc, char *argv[])
{
///////////
// init
    string name1 = argv[1];
    string name2 = argv[2];
    name = name1 + "_" + name2;
    string topic = "/calib/" + name + "/matches";
    string node_name = (string) "transform_estimation_" + name;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle node_te("/calib/" + name + "/transform_estimation");

    ros::Subscriber sub = nh.subscribe(topic, 1, &callback);

    dynamic_reconfigure::Server<calib::TransformEstimationConfig> server(node_te);
    dynamic_reconfigure::Server<calib::TransformEstimationConfig>::CallbackType f;
    f = boost::bind(&conf_callback, _1, _2);
    server.setCallback(f);

  ///////////
// main code
  
 
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