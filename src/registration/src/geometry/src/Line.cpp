#include <geometry/Line.h>

using namespace geometry;

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


/**
 * @brief Returns 2 points on the line to create a segment.
 * 
 * @param length The length of the segment.
 */
std::vector <geometry_msgs::Point> Line::get_points(float length)
{
    std::vector <geometry_msgs::Point> segment;
    geometry_msgs::Point p;
    p.x = this->point.getX() - 0.5 * this->direction.getX() * length;
    p.y = this->point.getY() - 0.5 * this->direction.getY() * length;
    p.z = this->point.getZ() - 0.5 * this->direction.getZ() * length;
    segment.push_back(p);
    p.x = this->point.getX() + 0.5 * this->direction.getX() * length;
    p.y = this->point.getY() + 0.5 * this->direction.getY() * length;
    p.z = this->point.getZ() + 0.5 * this->direction.getZ() * length;
    segment.push_back(p);

    return segment;
}