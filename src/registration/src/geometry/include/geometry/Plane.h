#ifndef GEOMETRY_PLANE
#define GEOMETRY_PLANE

// std
	#include <cmath>
// ros
	#include <ros/console.h>
// tf
	#include <tf/LinearMath/Vector3.h>
// My libraries
	#include <geometry/Line.h>

/**
 * @brief Class in charge of providing useful geometric fonctions for planes.
 * @details The class is capable of computing line intersection between planes.
 */

namespace geometry
{
	class Plane
	{
		private:
			tf::Vector3 normal;                   
			tf::Vector3 point;
			float d;                         

		public:
			Plane(tf::Vector3 normal, tf::Vector3 point);
			Plane(float* coef);
			Line intersect(Plane P);
	};
}

#endif