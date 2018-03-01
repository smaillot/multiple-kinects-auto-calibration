#ifndef GEOMETRY_LINE
#define GEOMETRY_LINE

// std
	#include <vector>
// ros
  	#include <visualization_msgs/Marker.h>
// tf
	#include <tf/LinearMath/Vector3.h>

/**
 * @brief Class in charge of providing useful geometric fonctions for lines.
 * @details 
 */

namespace geometry
{
	class Line
	{
		private:
			tf::Vector3 direction;
			tf::Vector3 point;

		public:
			Line(tf::Vector3 direction, tf::Vector3 point);
			std::vector <geometry_msgs::Point> get_points(float length);
	};
}

#endif