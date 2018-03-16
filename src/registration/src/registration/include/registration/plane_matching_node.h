// std
    #include <string>
    #include <iostream>
    #include <vector>
    #include <Eigen/Core>
    #include <math.h>
    #include <numeric>
// tf
	#include <tf/transform_listener.h>
    #include <tf/transform_broadcaster.h>
	#include <tf/LinearMath/Vector3.h>
    #include <tf/LinearMath/Matrix3x3.h>
    #include <tf_conversions/tf_eigen.h>
// custom
    #include <geometry/Line.h>
    #include <geometry/Plane.h>
    #include <geometry/PointCloud.h>

struct motion_t
{
    tf::Transform H;
    std::vector <float> residuals_angle;
    std::vector <float> residuals_translation;
};

class PCRegistered
{
	private:
		std::string sub_name;
		std::string pub_name;
		std::string frame;
		ros::NodeHandle node;
		tf::TransformListener tf_listener;
		ros::Subscriber pc_sub;
		ros::Publisher pc_pub;
        tf::TransformBroadcaster br;
		
	public:
		PCRegistered(std::string sub_name, std::string pub_name, std::string frame, ros::NodeHandle node);
		void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc);
};

bool tf_exists(tf::TransformListener* tf_listener, tf::StampedTransform* tf, std::string name);