// std
    #include <string>
    #include <iostream>
    #include <vector>
    #include <Eigen/Core>
// tf
	#include <tf/transform_listener.h>
    #include <tf/transform_broadcaster.h>
	#include <tf/LinearMath/Vector3.h>
    #include <tf/LinearMath/Matrix3x3.h>
// custom
    #include <geometry/Line.h>
    #include <geometry/Plane.h>
    #include <geometry/PointCloud.h>

struct motion_t
{
    tf::Matrix3x3 R;
    tf::Vector3 T;
    std::vector <float> residuals;
};

bool tf_exists(tf::TransformListener* tf_listener, tf::StampedTransform* tf, std::string name);