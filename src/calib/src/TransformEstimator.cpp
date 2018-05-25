#include "calib/TransformEstimator.h"

using namespace std;


/*
* @brief Add new points to the estimator.
*
* @param points Vector of points to add.
* @param source Whether these points are from the source (or target).
*/
void TransformEstimator::addPoints(vector<Eigen::Vector3f> points, bool source)
{
    vector<Eigen::Vector3f>* pts;
    if (!source)
    {
        for (int i = 0; i < points.size(); i++)
        {
            this->points_source.push_back(points[i]);
        }
    }
    else
    {
        for (int i = 0; i < points.size(); i++)
        {
            this->points_target.push_back(points[i]);
        }
    }
}

/*
* @brief Add new planes to the estimator.
*
* @param planes Vector of planes to add.
* @param source Whether these planes are from the source (or target).
*/
void TransformEstimator::addPlanes(vector<Eigen::Vector4f> planes, bool source)
{
    vector<Eigen::Vector4f>* p;
    if (!source)
    {
        p = &(this->planes_source);
    }
    else
    {
        p = &(this->planes_target);
    }
    for (int i = 0; i < planes.size(); i++)
    {
        p->push_back(planes[i]);
    }
    while (this->planes_weights.size() < p->size())
    {
        this->planes_weights.push_back(1);
    }
}

void TransformEstimator::addPlanes(vector<Eigen::Vector4f> planes, bool source, float w)
{
    vector<Eigen::Vector4f>* p;
    if (!source)
    {
        p = &(this->planes_source);
    }
    else
    {
        p = &(this->planes_target);
    }
    for (int i = 0; i < planes.size(); i++)
    {
        p->push_back(planes[i]);
    }
    while (this->planes_weights.size() < p->size())
    {
        this->planes_weights.push_back(w);
    }
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
* @brief Estimate rotation.
*/
Eigen::Matrix3f TransformEstimator::getRotation()
{
    Eigen::Matrix3f K = this->computeRotCorr();
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(K, Eigen::ComputeFullV | Eigen::ComputeFullU);
	Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();
    Eigen::DiagonalMatrix<float, 3> D(1, 1, R.determinant());
	R = svd.matrixU() * D * svd.matrixV().transpose();

    return R;
}

/*
* @brief Estimate translation.
*
* @param R Estimated rotation.
*/
Eigen::Vector3f TransformEstimator::getTranslation(Eigen::Matrix3f R)
{
    if (this->isValid())
    {
        int M = this->points_source.size();
        int N = this->planes_source.size();
        Eigen::Vector3f t;
        
        Eigen::Matrix3f A1;
        Eigen::Vector3f b1;
        Eigen::MatrixX3f A2(N, 3);
        Eigen::VectorXf b2(N);

        if (M > 0) // if there is point matches
        {
            A1 = Eigen::Matrix3f::Identity(3, 3);
            A1 *= M;
            b1 = this->computeCentroid(this->points_source) - R * this->computeCentroid(this->points_target);
            b1 *= M;
        }
        if (N > 0) // if there is plane matches
        {
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    A2(i, j) = planes_weights[i] * this->planes_target[i](j);
                }
                b2(i) = planes_weights[i] * (this->planes_target[i](3) - this->planes_source[i](3));
            }
        }

        // stack A and b and compute t
        if (N > 0)
        {
            if (M > 0) // both planes and points
            {
                Eigen::MatrixX3f A(N+3, 3);
                Eigen::VectorXf b(N+3);
                A << A1, A2;
                b << b1, b2;

                t = (A.transpose() * A).inverse() * A.transpose() * b;
            }
            else // only planes
            {
                t = (A2.transpose() * A2).inverse() * A2.transpose() * b2;
            }
        }
        else // only points
        {
            t = (A1.transpose() * A1).inverse() * A1.transpose() * b1;
        }
        return t;
    }
}

/*
* @brief Estimate transform.
*/
Eigen::Affine3d TransformEstimator::getTransform(bool transl, bool rot)
{
    ROS_INFO_STREAM("Compute transform with:");
    ROS_INFO_STREAM("\t" << points_source.size() << " points:");
    for (int i = 0; i < this->points_source.size(); i++)
    {
        ROS_INFO_STREAM("\t\t" << this->points_source[i].transpose() << " -> " << this->points_target[i].transpose());
    }
    ROS_INFO_STREAM("\t" << planes_source.size() << " planes:");
    for (int i = 0; i < this->planes_source.size(); i++)
    {
        ROS_INFO_STREAM("\t\t" << this->planes_source[i].transpose() << " -> " << this->planes_target[i].transpose());
    }
    Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R = this->getRotation();
    H.topLeftCorner(3, 3) = R;
    H.topRightCorner(3, 1) = this->getTranslation(R);
    ROS_INFO_STREAM("transform =" << endl << H);
    Eigen::Affine3d T;
    T.matrix() = H.cast<double>();
    return T;
}

/*
* @brief Estimate transform.
*/
Eigen::Affine3d TransformEstimator::getTransform()
{
    return this->getTransform(true, true);
}

/*
* @brief Compute the centroid of a point set.
*
* @param vec Input vectors.
*/
Eigen::Vector3f TransformEstimator::computeCentroid(vector <Eigen::Vector3f> vec)
{
    Eigen::Vector3f centroid;
    int n = vec.size();
    for (int i = 0; i < n; i++)
    {
        centroid += vec[i];
    }
    centroid /= (float) n;
    return centroid;
}

/*
* @brief Compute the correlation matrix for rotation estimation.
*/
Eigen::Matrix3f TransformEstimator::computeRotCorr()
{
    Eigen::Matrix3f K = Eigen::Matrix3f::Zero(3, 3);
    Eigen::Vector3f ps = this->computeCentroid(this->points_source);
    Eigen::Vector3f pt = this->computeCentroid(this->points_target);
    if (this->isValid())
    {
        if (this->points_source.size() > 0)
        {
            for (int i = 0; i < this->points_source.size(); i++)
            {
                K += (this->points_source[i] - ps) * (this->points_target[i] - pt).transpose();
            }
        }
        if (this->planes_source.size() > 0)
        {
            Eigen::Vector3f Ns;
            Eigen::Vector3f Nt;
            for (int i = 0; i < this->planes_source.size(); i++)
            {
                Ns = this->planes_source[i].block(0, 0, 3, 1);
                Nt = this->planes_target[i].block(0, 0, 3, 1);
                K += (Ns * Nt.transpose()) * this->planes_weights[i];
            }
        }
    }
    return K;
}

/*
* @brief Check if the inputs provided are sufficient to estimate transform.
*/
bool TransformEstimator::isValid()
{
    if (points_source.size() != points_target.size()) {return false;};
    if (planes_source.size() != planes_target.size()) {return false;};
    if (planes_source.size() != planes_weights.size()) {return false;};
    if (planes_source.size() + points_source.size() < 3) {return false;};
    return true;
}

// int main(int argc, char *argv[])
// {
//     TransformEstimator Obj;
//     vector<Eigen::Vector3f> pts;
//     vector<Eigen::Vector3f> ptt;
//     vector<Eigen::Vector4f> pls;
//     vector<Eigen::Vector4f> plt;
//     pts.push_back(Eigen::Vector3f(0.0, 0.0, 1.0));
//     ptt.push_back(Eigen::Vector3f(0.0, 0.0, 2.05));
//     pts.push_back(Eigen::Vector3f(0.0, 0.0, -1.0));
//     ptt.push_back(Eigen::Vector3f(0.0, 0.0, 0.1));
//     pls.push_back(Eigen::Vector4f(1.0, 0.01, 0.0, 0.0));
//     plt.push_back(Eigen::Vector4f(0.7, 0.7, 0.0, 0.0));
//     pls.push_back(Eigen::Vector4f(0.05, 1.0, 0.0, 0.0));
//     plt.push_back(Eigen::Vector4f(-0.7, 0.71, 0.0, 0.0));
//     Obj.addPlanes(pls, true);
//     Obj.addPlanes(plt, false);
//     Obj.addPoints(pts, true);
//     Obj.addPoints(ptt, false);
//     Obj.getTransform();

//     return 0;
// }