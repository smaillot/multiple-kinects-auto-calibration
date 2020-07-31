#include "calib/TransformEstimator.h"

using namespace std;

void TransformEstimator::conf_callback(calib::TransformEstimationConfig &config, uint32_t level)
{
    this->weight = config.planes_weight;

    this->use_points = config.use_points;
    this->use_planes = config.use_planes;
    this->compute_rotation = config.compute_rotation;
    this->compute_translation = config.compute_translation;
    this->project_points = config.project_points;
}

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
            ROS_DEBUG_STREAM("Adding point: " << points[i].transpose());
            this->points_source.push_back(this->project(points[i], source));
        }
    }
    else
    {
        for (int i = 0; i < points.size(); i++)
        {
            ROS_DEBUG_STREAM("Adding point: " << points[i].transpose());
            this->points_target.push_back(this->project(points[i], source));
        }
    }
}

Eigen::Vector3f TransformEstimator::project(Eigen::Vector3f point, bool source)
{
    Eigen::Vector3f proj = point;
    if (this->planes_source.size() == 2 && this->planes_target.size() == 2)
    {
        std::vector<Eigen::Vector4f> planes;
        if (source)
        {
            planes = this->planes_source;
        }
        else
        {
            planes = this->planes_target;
        }
        Eigen::Vector3f normal1(planes[0][0], planes[0][1], planes[0][2]);
        Eigen::Vector3f normal2(planes[1][0], planes[1][1], planes[1][2]);
        Eigen::Vector3f intersec = normal1.cross(normal2);
        ROS_DEBUG_STREAM("\tProject on " << intersec.transpose());
        proj = point.dot(intersec) * intersec;
        ROS_DEBUG_STREAM("\tResult: " << proj.transpose());
    }
    return proj;
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
        ROS_DEBUG_STREAM("Adding plane: " << planes[i].transpose() << " with weight " << 1);
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
        ROS_DEBUG_STREAM("Adding plane: " << planes[i].transpose() << " with weight " << w);
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
    ROS_DEBUG("Reset matches");
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
    ROS_DEBUG_STREAM("Rotation:" << endl << R);
    Eigen::DiagonalMatrix<float, 3> D(1, 1, R.determinant());
	R = svd.matrixU() * D * svd.matrixV().transpose();
    ROS_DEBUG_STREAM("Regularized rotation:" << endl << R);

    return R;
}

/*
* @brief Estimate translation.
*
* @param R Estimated rotation.
*/
Eigen::Vector3f TransformEstimator::getTranslation(Eigen::Matrix3f R)
{
    Eigen::Vector3f t;
    if (this->isValid())
    {
        int M = this->points_source.size();
        int N = this->planes_source.size();
        
        Eigen::Matrix3f A1;
        Eigen::Vector3f b1;
        Eigen::MatrixX3f A2(N, 3);
        Eigen::VectorXf b2(N);

        if (M > 0) // if there is point matches
        {
            A1 = Eigen::Matrix3f::Identity(3, 3);
            A1 *= M;
            b1 = this->computeCentroid(this->points_target) - R * this->computeCentroid(this->points_source);
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
                b2(i) = planes_weights[i] * (this->planes_source[i](3) - this->planes_target[i](3));
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
                ROS_DEBUG_STREAM("A =" << endl << A);
                ROS_DEBUG_STREAM("b =" << endl << b);

                t = (A.transpose() * A).inverse() * A.transpose() * b;
            }
            else // only planes
            {
                ROS_DEBUG_STREAM("A =" << endl << A2);
                ROS_DEBUG_STREAM("b =" << endl << b2);
                t = (A2.transpose() * A2).inverse() * A2.transpose() * b2;
            }
        }
        else // only points
        {
            ROS_DEBUG_STREAM("A =" << endl << A1);
            ROS_DEBUG_STREAM("b =" << endl << b1);
            t = (A1.transpose() * A1).inverse() * A1.transpose() * b1;
        }

        ROS_DEBUG_STREAM("t =" << endl << t);
    }
    return t;
}

/*
* @brief Estimate transform.
*/
Eigen::Affine3d TransformEstimator::getTransform(bool transl, bool rot)
{
    if (this->planes_source.size() == 2 && this->planes_target.size() == 2 && this->points_source.size() == 0 && this->points_target.size() == 0)
    {
        std::vector<Eigen::Vector3f> zero;
        zero.push_back(Eigen::Vector3f(0,0,0));
        this->addPoints(zero, true);
        this->addPoints(zero, false);
    }

    ROS_DEBUG_STREAM("Compute transform with:");
    ROS_DEBUG_STREAM("\t" << points_source.size() << " points:");
    for (int i = 0; i < this->points_source.size(); i++)
    {
        ROS_DEBUG_STREAM("\t\t" << this->points_source[i].transpose() << " -> " << this->points_target[i].transpose());
    }
    ROS_DEBUG_STREAM("\t" << planes_source.size() << " planes:");
    for (int i = 0; i < this->planes_source.size(); i++)
    {
        ROS_DEBUG_STREAM("\t\t" << this->planes_source[i].transpose() << " -> " << this->planes_target[i].transpose());
    }
    Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    if (rot)
    {
        R = this->getRotation();
        H.topLeftCorner(3, 3) = R;
    }
    if (transl)
    {
        H.topRightCorner(3, 1) = this->getTranslation(R);
    }
    ROS_DEBUG_STREAM("transform =" << endl << H);
    Eigen::Affine3d T;
    T.matrix() = H.cast<double>();
    ROS_DEBUG_STREAM("H =" << endl << H);   
    ROS_INFO_STREAM("Compute transform with " << points_source.size() << " points and " << planes_source.size() << " planes."); 
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
    Eigen::Vector3f centroid(0, 0, 0);
    int n = vec.size();
    for (int i = 0; i < n; i++)
    {
        ROS_DEBUG_STREAM("\tAdding\t" << vec[i].transpose());
        ROS_DEBUG_STREAM("\t\tCurrent centroid\t" << centroid.transpose());
        centroid += vec[i];
    }
    centroid /= (float) n;
    ROS_DEBUG_STREAM("Points centroid: " << centroid.transpose());
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
                K += (this->points_target[i] - pt) * (this->points_source[i] - ps).transpose();
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
                K += (Nt * Ns.transpose()) * this->planes_weights[i];
            }
        }
    }
    ROS_DEBUG_STREAM("rotation correlation =" << endl << K);
    return K;
}

/*
* @brief Check if the inputs provided are sufficient to estimate transform.
*/
bool TransformEstimator::isValid()
{
    bool valid = true;
    if (this->points_source.size() != this->points_target.size()) {valid = false;};
    if (this->planes_source.size() != this->planes_target.size()) {valid = false;};
    if (this->planes_source.size() != this->planes_weights.size()) {valid = false;};
    if (this->planes_source.size() + this->points_source.size() < 3) {valid = false;};
    if (valid)
    {
        ROS_DEBUG("Valid inputs !");
    }
    else
    {
        ROS_ERROR_STREAM("Invalid input for transform estimation:");
        ROS_ERROR_STREAM("\tPoints: " << points_source.size() << ", " << points_target.size());
        ROS_ERROR_STREAM("\tPlanes: " << planes_source.size() << ", " << planes_target.size());
        ROS_ERROR_STREAM("\tWeights: " << planes_weights.size());
    }
    return valid;
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