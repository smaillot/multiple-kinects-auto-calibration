// std
    #include <string>
    #include <iostream>
    #include <ctime>
// tf
	#include <tf/transform_listener.h>
// PCL specific includes
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl_ros/transforms.h>
	#include <pcl/common/transforms.h>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}