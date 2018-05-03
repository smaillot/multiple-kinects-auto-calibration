#include <string>
#include <iostream>
#include <vector>
#include <geometry/Plane.h>
#include <Eigen/SVD> 
#include <Eigen/Core>
#include <Eigen/LU>
#include <ros/console.h>


class TransformEstimator
{
    private:
        
        std::vector<float> planes_weights;
        std::vector<Eigen::Vector3f> points_source;
        std::vector<Eigen::Vector3f> points_target;
        std::vector<Eigen::Vector4f> planes_source;
        std::vector<Eigen::Vector4f> planes_target;

    protected:

        bool isValid();
        Eigen::Matrix3f computeRotCorr();
        Eigen::Vector3f computeCentroid(std::vector <Eigen::Vector3f> vec);

    public:

        void addPoints(std::vector<Eigen::Vector3f> points, bool source);
        void addPlanes(std::vector<Eigen::Vector4f> planes, bool source);
        void addPlanes(std::vector<geometry::Plane> planes, bool source);
        void reset();
        Eigen::Matrix3f getRotation();
        Eigen::Vector3f getTranslation(Eigen::Matrix3f R);
        Eigen::Matrix4f getTransform();
};