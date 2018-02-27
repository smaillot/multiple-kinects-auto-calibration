#include <geometry/Plane.h>

/**
 * @brief Plane class constructor defining plane from a point and a normal vector.
 * 
 * @param normal Normal vector to the plane.
 * @param point Any point in the plane.
 */

 using namespace geometry;
 
Plane::Plane(tf::Vector3 normal, tf::Vector3 point)
{
    this->normal = normal.normalized();
    this->point = point;
    this->d = -(normal.dot(point));
}

/**
 * @brief Plane class constructor defining plane from its coefficients.
 * 
 * @param coef Array of 4 coefficients defining the plane st. aX+bY+cZ+d=0
 */
Plane::Plane(float* coef)
{
    this->d = coef[3];
    this->normal = tf::Vector3(coef[0], coef[1], coef[2]).normalized();
    this->point = normal * std::abs(d) / normal.length();
}

/**
 * @brief Compute intersection with a second plane.
 * 
 * @param P Second plane.
 */
Line Plane::intersect(Plane P)
{
    tf::Vector3 n1 = this->normal;
    tf::Vector3 n2 = P.normal;
    float d1 = this->d;
    float d2 = P.d;

    // find direction vector of the intersection line
    tf::Vector3 v = n1.cross(n2);                   

    // if |direction| = 0, 2 planes are parallel (no intersect)
    // return a line with NaN
    if(v.getX() == 0 && v.getY() == 0 && v.getZ() == 0)
        return Line(tf::Vector3(NAN, NAN, NAN), tf::Vector3(NAN, NAN, NAN));

    // find a point on the line, which is also on both planes
    // choose simplest plane where d=0: ax + by + cz = 0
    float dot = v.dot(v);                       // V dot V
    tf::Vector3 u1 =  d2 * n1;                      // d2 * N1
    tf::Vector3 u2 = -d1 * n2;                      //-d1 * N2
    tf::Vector3 p = (u1 + u2).cross(v) / dot;       // (d2*N1-d1*N2) X V / V dot V

    ROS_DEBUG_STREAM("Plane 1: normal = (" << this->normal.getX() << ", + " << this->normal.getY() << ", + " << this->normal.getZ() << "), point = (" << this->point.getX() << ", + " << this->point.getY() << ", + " << this->point.getZ() << ")");
    ROS_DEBUG_STREAM("Plane 2: normal = (" << P.normal.getX() << ", + " << P.normal.getY() << ", + " << P.normal.getZ() << "), point = (" << P.point.getX() << ", + " << P.point.getY() << ", + " << P.point.getZ() << ")");
    ROS_DEBUG_STREAM("Line detected: direction = (" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "), point =  (" << p.getX() << ", " << p.getY() << ", " << p.getZ() << ")");

    return Line(v, p);
}