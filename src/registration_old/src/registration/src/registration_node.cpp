// std
	#include <string>
	#include <iostream>
	#include <math.h>
    #include <Eigen/Core>
// ros
	#include <ros/console.h>
	#include <ros/callback_queue.h>
	#include <ros/callback_queue_interface.h>
// point cloud
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/impl/point_types.hpp>
	#include <pcl_ros/transforms.h>
	#include <pcl/filters/voxel_grid.h>
	#include <pcl/filters/passthrough.h>
	#include <pcl/filters/radius_outlier_removal.h>
	#include <pcl/filters/statistical_outlier_removal.h>
	#include <pcl/keypoints/iss_3d.h>
	#include <pcl/keypoints/sift_keypoint.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl/kdtree/kdtree_flann.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/registration/correspondence_estimation.h>
    #include <pcl/registration/transformation_estimation_svd.h>
    #include <pcl/registration/correspondence_rejection_one_to_one.h>
    #include <pcl/registration/correspondence_rejection_sample_consensus.h>
    #include <pcl/features/vfh.h>
    #include <pcl/features/fpfh.h>
    #include <pcl/features/shot.h>
    #include <pcl/filters/extract_indices.h>
// tf
	#include <tf/LinearMath/Transform.h>
	#include <tf_conversions/tf_eigen.h>
	#include <tf/transform_listener.h>
	#include <tf/transform_broadcaster.h>
// synchronization
	#include <message_filters/subscriber.h>
	#include <message_filters/time_synchronizer.h>
	#include <message_filters/sync_policies/approximate_time.h>
// custom
	#include <geometry/Line.h>
	#include <geometry/Plane.h>
	#include <geometry/PointCloud.h>
	#include <registration/TransformEstimator.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

vector<string> inputs;
string output;
// ros::Publisher pub;

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

pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_PCL(const PointCloud2ConstPtr& msgPtr)
{
    PointCloud2 msg = *msgPtr;
    pcl::PCLPointCloud2* pc2 = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr pc2Ptr(pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_conversions::toPCL(msg, *pc2);
    pcl::fromPCLPointCloud2(*pc2Ptr, *pclPtr);

    return pclPtr;
}

string get_topic_name(string input, int n)
{
    return "/reconstruction/planes/" + input + "/plane" + patch::to_string(n+1);
}

vector<geometry::Plane> get_planes(string input, tf::TransformListener* tf_listener)
{
    tf::StampedTransform transf; 
	vector <geometry::Plane> planes;

	for (int i = 0; i < 2; i++)
	{
        if (tf_exists(tf_listener, &transf, get_topic_name(input, i)))
        {
            tf_listener->lookupTransform("cam_center", get_topic_name(input, i), ros::Time::now(), transf);
            planes.push_back(geometry::Plane(transf));
        }
	}

    return planes;
}

void pc_callback(const PointCloud2ConstPtr& pc1, const PointCloud2ConstPtr& pc2, tf::TransformListener* tf_listener)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input2(new pcl::PointCloud<pcl::PointXYZ>);
    input1 = convert_to_PCL(pc1);
    input2 = convert_to_PCL(pc2);

    vector<geometry::Plane> planes1 = get_planes(inputs[0], tf_listener);
    vector<geometry::Plane> planes2 = get_planes(inputs[1], tf_listener);

    TransformEstimator* TE = new TransformEstimator();
    vector<Eigen::Vector3f> vec3;
    for (int i = 0; i < input1->points.size(); i++)
    {
        vec3.push_back(Eigen::Vector3f(input1->points[i].x, input1->points[i].y, input1->points[i].z));
    }
    if (vec3.size() > 0)
    {
        TE->addPoints(vec3, true);
    }
    vec3.clear();
    for (int i = 0; i < input2->points.size(); i++)
    {
        vec3.push_back(Eigen::Vector3f(input2->points[i].x, input2->points[i].y, input2->points[i].z));
    }
    if (vec3.size() > 0)
    {
        TE->addPoints(vec3, false);
    }

    if (planes1.size() > 0)
    {
        TE->addPlanes(planes1, true);
    }
    if (planes2.size() > 0)
    {
        TE->addPlanes(planes2, false);
    }

    tf::Transform tf;
    if (TE->isValid())
    {
        Eigen::Matrix4f transf = TE->getTransform();
        ROS_INFO_STREAM("Transform:\n" << transf);
        tf::Vector3 origin;
        tf::Matrix3x3 tf3d;
        tf::Quaternion tfqt;

        origin.setValue(static_cast<double>(transf(0,3)),static_cast<double>(transf(1,3)),static_cast<double>(transf(2,3)));
        tf3d.setValue(static_cast<double>(transf(0,0)), static_cast<double>(transf(0,1)), static_cast<double>(transf(0,2)), 
                        static_cast<double>(transf(1,0)), static_cast<double>(transf(1,1)), static_cast<double>(transf(1,2)), 
                        static_cast<double>(transf(2,0)), static_cast<double>(transf(2,1)), static_cast<double>(transf(2,2)));
        tf3d.getRotation(tfqt);
        tf.setOrigin(origin);
        tf.setRotation(tfqt);
    }

    static tf::TransformBroadcaster br; 
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "cam_center", "/" + inputs[0] + "_" + inputs[1]));
}

int main(int argc, char *argv[])
{
    inputs.push_back(argv[1]);
    inputs.push_back(argv[2]);
    output = argv[3];

    // Initialize ROS
        string node_name = "registration_" + inputs[0] + "_" + inputs[1];
        ros::init(argc, argv, node_name);
        ros::NodeHandle nh;
        tf::TransformListener* tf_listener;

    // Synchronize both kinects messages
        message_filters::Subscriber<PointCloud2> cam1(nh, "reconstruction/point_clouds/" + inputs[0] + "/keypoints_matches", 1);
        message_filters::Subscriber<PointCloud2> cam2(nh, "reconstruction/point_clouds/" + inputs[1] + "/keypoints_matches", 1);
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2> KinectSync;
        Synchronizer<KinectSync> sync(KinectSync(10), cam1, cam2);
        sync.registerCallback(boost::bind(&pc_callback, _1, _2, tf_listener));

        // pub = nh.advertise<sensor_msgs::PointCloud2>(path + "/" + output, 1);

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}