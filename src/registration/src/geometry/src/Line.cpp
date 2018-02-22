#include <geometry/Line.h>

/**
 * @brief Line class constructor defining line from a point and direction.
 * 
 * @param direction Vector pointing in the line direction.
 * @param point Any point in the line.
 */
Line::Line(tf::Vector3 direction, tf::Vector3 point)
{
    this->direction = direction.normalized();
    this->point = point;
}