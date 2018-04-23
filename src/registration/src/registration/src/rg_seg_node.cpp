#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

using namespace std;

string input;
int k;
int min_cl;
int max_cl;
int neigh;
float sth;
float cth;
float frequency;
string path = "/reconstruction/point_clouds/";
ros::Subscriber sub;
ros::Publisher pub;


void callback(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg (*pc, input_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&input_cloud);
    
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, indices);

    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(k);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(min_cl);
    reg.setMaxClusterSize(max_cl);

    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);

    reg.setSmoothnessThreshold (sth / 180.0 * M_PI);
    reg.setCurvatureThreshold (cth);

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    ROS_DEBUG_STREAM("Number of clusters is equal to " << clusters.size ());

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    sensor_msgs::PointCloud2 msg;
    pcl::PCLPointCloud2* output(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*colored_cloud, *output);
    pcl_conversions::fromPCL(*output, msg);

	pub.publish(msg);
}

int main(int argc, char *argv[])
{
    input = argv[1];
    string f = argv[2];
    k = (int) atof(f.c_str());
    f = argv[3];
    min_cl = (int) atof(f.c_str());
    f = argv[4];
    max_cl = (int) atof(f.c_str());
    f = argv[5];
    neigh = (int) atof(f.c_str());
    f = argv[6];
    sth = (float) atof(f.c_str());
    f = argv[7];
    cth = (float) atof(f.c_str());
    f = argv[8];
    frequency = (float) atof(f.c_str());
    
	// Initialize ROS
		string node_name = "region_growing_segmentation_" + input;
		ros::init(argc, argv, node_name);
		ros::NodeHandle nh;

		sub = nh.subscribe(path+input, 1, &callback);
        pub = nh.advertise<sensor_msgs::PointCloud2>("/reconstruction/planes/" + input, 1);

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
	}

	return 0;
}