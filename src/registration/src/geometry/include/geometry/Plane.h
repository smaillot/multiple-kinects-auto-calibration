#ifndef GEOMETRY_PLANE
#define GEOMETRY_PLANE

// std
	#include <cmath>
	#include <iostream>
// ros
	#include <ros/console.h>
// tf
	#include <tf/LinearMath/Vector3.h>
	#include <tf/transform_listener.h>
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
			friend std::ostream& operator<<(std::ostream&, const Plane&);
		public:
			tf::Vector3 normal;                   
			tf::Vector3 point;
			float d;                         

			Plane(tf::Vector3 normal, tf::Vector3 point);
			Plane(tf::Transform transform);
			Plane(float* coef);
			Line intersect(Plane P);
	};
}

#endif