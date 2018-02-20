// std
	#include <string>
//pcl
	#include <pcl_conversions/pcl_conversions.h>
	#include <pcl/point_cloud.h>
	#include <pcl/impl/point_types.hpp>
	#include <sensor_msgs/PointCloud2.h>

struct cutting_dim_t
{
	bool enable;
	float bounds[2];
}
struct cutting_t
{
	cutting_dim_t x;
	cutting_dim_t y;
	cutting_dim_t z;
}

class PointCloud
{
	private:
		std::string name;
		pcl::PCLPointCloud2* cloud;

		// parameters
			// subsampling
				double subsize;
			//cutting
				cutting_t cutting;
			// filtering
				bool filtering;
				double filter_radius;
				int filter_min_neighbors;
	public:
		// constructors
			PointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
			PointCloud(pcl::PCLPointCloud2& cloud);
			PointCloud(sensor_msgs::PointCloud2& cloud);
};