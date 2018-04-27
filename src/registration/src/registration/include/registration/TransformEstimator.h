#include <string>
#include <vector>
#include <geometry/Plane>
#include <Eigen/Core>


class TransformEstimator
{
    private:
        
        vector<int> planes_weights;
        vector<Eigen::Vector3d> points_source;
        vector<Eigen::Vector3d> points_target;
        vector<Eigen::Vector4d> planes_source;
        vector<Eigen::Vector4d> planes_target;

    protected:

        bool isValid();
        Eigen::Matrix3d computeRotCorr();

    public:

        TransformEstimator();
        void addPoints(vector<Eigen::Vector3d> points, bool source);
        void addPlanes(vector<Eigen::Vector4d> planes, bool source);
        void addPlanes(vector<geometry::Planes> planes, bool source);
        void reset();
        void setWeights(vector <float> weights);
        Eigen::Matrix3d getRotation();
        Eigen::Vector3d getTranslation();
        Eigen::Vector4d getTransform();
};