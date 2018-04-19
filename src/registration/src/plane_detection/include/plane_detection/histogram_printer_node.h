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

class Array3D {
    size_t m_width, m_height;
    std::vector<int> m_data;
  public:
    Array3D(size_t x, size_t y, size_t z, int init = 0):
      m_width(x), m_height(y), m_data(x*y*z, init)
    {}
    int& operator()(size_t x, size_t y, size_t z) {
        return m_data.at(x + y * m_width + z * m_width * m_height);
    }
};