// tf
	#include <tf/LinearMath/Vector3.h>

/**
 * @brief Class in charge of providing useful geometric fonctions for lines.
 * @details 
 */

class Line
{
	private:
		tf::Vector3 direction;
		tf::Vector3 point;

	public:
		Line(tf::Vector3 direction, tf::Vector3 point);
};