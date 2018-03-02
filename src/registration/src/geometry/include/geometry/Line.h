#ifndef GEOMETRY_LINE
#define GEOMETRY_LINE

// std
	#include <vector>
// ros
  	#include <visualization_msgs/Marker.h>
// tf
	#include <tf/LinearMath/Vector3.h>
	#include <tf/transform_listener.h>

/**
 * @brief Class in charge of providing useful geometric fonctions for lines.
 * @details 
 */

namespace geometry
{
	class Line
	{
		public:
			tf::Vector3 direction;
			tf::Vector3 point;

			Line(tf::Vector3 direction, tf::Vector3 point);
			std::vector <geometry_msgs::Point> get_points(float length);
			tf::Transform get_transform(geometry::Line line);
	};
}

#endif