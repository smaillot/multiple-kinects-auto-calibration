#ifndef GEOMETRY_PLANE_DETECTOR
#define GEOMETRY_PLANE_DETECTOR

// std
    #include <string>
    #include <iostream>
// ros
    #include <ros/console.h>
// My libraries
    #include <geometry/Line.h>
    #include <geometry/Plane.h>
    #include <geometry/PointCloud.h>

    namespace geometry
    {
        class PlaneDetector
        {
            friend class PointCloud;
            private:
                // ros node interaction
                    ros::NodeHandle node;  
                    const tf::TransformListener* tf_listener;

                    std::string sub_name;  
                    std::string pub_name;
                    ros::Subscriber pc_sub;
                    ros::Publisher pc_pub;

                // Point cloud
                    PointCloud point_cloud;
            public:

        };
    };

#endif