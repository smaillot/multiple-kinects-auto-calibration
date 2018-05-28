#include <string>
#include <iostream>
#include <vector>
#include <Eigen/SVD> 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>


class TransformEstimator
{
    private:
        
        std::vector<float> planes_weights;
        std::vector<Eigen::Vector3f> points_source;
        std::vector<Eigen::Vector3f> points_target;
        std::vector<Eigen::Vector4f> planes_source;
        std::vector<Eigen::Vector4f> planes_target;

    protected:

        Eigen::Matrix3f computeRotCorr();
        Eigen::Vector3f computeCentroid(std::vector <Eigen::Vector3f> vec);

    public:

        bool isValid();
        void addPoints(std::vector<Eigen::Vector3f> points, bool source);
        void addPlanes(std::vector<Eigen::Vector4f> planes, bool source);
        void addPlanes(std::vector<Eigen::Vector4f> planes, bool source, float w);
        void reset();
        Eigen::Matrix3f getRotation();
        Eigen::Vector3f getTranslation(Eigen::Matrix3f R);
        Eigen::Affine3d getTransform();
        Eigen::Affine3d getTransform(bool transl, bool rot);
};