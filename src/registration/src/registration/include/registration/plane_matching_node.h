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

bool tf_exists(tf::TransformListener* tf_listener, tf::StampedTransform* tf, std::string name);