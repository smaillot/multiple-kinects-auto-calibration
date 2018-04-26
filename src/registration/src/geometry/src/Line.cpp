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



/**
* @brief Compute the transform between 2 frames given a identiaue line equation in each frame. 
* One translation is still undetermine (in the direction of the current line), this is fixed by finding the transformatin that leads to the smallest translation.
*
* @params line The line object in a different reference frame.
*/
tf::Transform Line::get_transform(geometry::Line line)
{
    tf::Vector3 v1 = this->direction;
    tf::Vector3 v2 = line.point - this->point;
    tf::Vector3 transl = -v2 - v1.dot(v2) / v1.length() * v1;
    tf::Vector3 axis = v1.cross(v2);
    float angle = v1.angle(v2);
    tf::Quaternion Q(axis, angle);

    // convert into tf
        tf::Transform transform;
        transform.setOrigin(transl);
        transform.setRotation(Q);

    return transform;
}