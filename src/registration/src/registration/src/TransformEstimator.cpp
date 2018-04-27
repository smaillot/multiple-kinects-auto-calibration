#include "registration/TransformEstimator.h"

TransformEstimator::TransformEstimator()
{

}

/*
* @brief Add new points to the estimator.
*
* @param points Vector of points to add.
* @param source Whether these points are from the source (or target).
*/
void TransformEstimator::addPoints(vector<Eigen::Vector3d> points, bool source)
{
    vector<Eigen::Vector3d>* points;
    if (source)
    {
        points = &(this->points_source);
    }
    else
    {
        points = &(this->points_target);
    }
    for (int i = 0; i < points.size(); i++)
    {
        points->push_back(points[i]);
    }
}

/*
* @brief Add new planes to the estimator.
*
* @param planes Vector of planes to add.
* @param source Whether these planes are from the source (or target).
*/
void TransformEstimator::addPlanes(vector<Eigen::Vector4d> planes, bool source)
{
    vector<Eigen::Vector3d>* planes;
    if (source)
    {
        planes = &(this->planes_source);
    }
    else
    {
        planes = &(this->planes_target);
    }
    for (int i = 0; i < points.size(); i++)
    {
        planes->push_back(points[i]);
        planes_weights.push_back(1);
    }
}

/*
* @brief Add new planes to the estimator.
*
* @param planes Vector of planes to add (using Plane object representation).
* @param source Whether these planes are from the source (or target).
*/
void TransformEstimator::addPlanes(vector <geometry::Planes> planes, bool source)
{
    vector <Eigen::Vector4d> vec;
    for (int i = 0; i < planes.size(); i++)
    {
        vec.push_back(geometry::Plane(planes[i]));
    }
    this->addPlanes(vec, source);
}

/*
* @brief Reset object vectors.
*/
void TransformEstimator::reset()
{
    this->planes_weights.clear();
    this->points_source.clear();
    this->points_target.clear();
    this->planes_source.clear();
    this->planes_target.clear();
}

/*
* brief Estimate rotation.
*/
Eigen::Matrix3d TransformEstimator::getRotation()
{
    Eigen::Matrix3d K = this->computeRotCorr();
    JacobiSVD<Matrix3d> svd(Rhat, ComputeFullV | ComputeFullU);
	Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    DiagonalMatrix<double, 3> D(1, 1, R.determinant());
	Matrix3d R = svd.matrixU() * D * svd.matrixV().transpose();

    return R;
}

/*
* brief Estimate translation.
*/
Eigen::Vector3d TransformEstimator::getTranslation(Eigen::Matrix3d R)
{
    if (this->isValid())
    {
        int M = this->points_source.size();
        int N = this->planes_source.size();
        if (N > 0)
        {
            Eigen::Matrix3d A1 = Matrix3d::Identity(3, 3);
            A1 *= M;
            b1 = this->computeCentroid(this->points_source) - R * this->computeCentroid(this->points_target);
            b1 *= M;
        }
        if (N > 0)
        {
            Eigen::MatrixX3d A2(N, 3);
            Eigen::VectorXd b2(N);
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    A2(i, j) = planes_weights[i] * this->planes_target[i](j);
                }
                b2(i) = planes_weights[i] * (this->planes_target[i](3) - this->planes_source[i](3));
            }
        }

        if (N > 0)
        {
            // both planes and points
            // stack A and b
            Eigen::MatrixX3d A(N+3, 3);
            Eigen::VectorXd b(N+3);
        }
        else
        {
            // only planes (because we assume its valid configuration)
            Eigen::MatrixX3d A(N, 3);
            Eigen::VectorXd b(N);
        }

        // t = ...
        return t;
    }
    else
    {
        return;
    }
}

/*
* brief Estimate transform.
*/
Eigen::Vector4d TransformEstimator::getTransform()
{

}

