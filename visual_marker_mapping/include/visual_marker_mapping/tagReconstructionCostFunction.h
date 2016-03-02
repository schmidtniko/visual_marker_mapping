#ifndef VISUAL_MARKER_MAPPING_TAGRECONSTRUCTIONCOSTFUNCTION_H_
#define VISUAL_MARKER_MAPPING_TAGRECONSTRUCTIONCOSTFUNCTION_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace camSurv
{
struct OpenCVReprojectionError
{
    OpenCVReprojectionError(const Eigen::Vector2d& observation,
        const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K)
        : observation(observation)
        , d(d)
        , K(K)
    {
    }

    template <typename T>
    bool operator()(const T* const camera_xyz, const T* const camera_q, const T* const point_xyz,
        T* residuals) const
    {

        // transform into camera coordinate system
        T tagCorC[3];
        ceres::UnitQuaternionRotatePoint(camera_q, point_xyz, tagCorC);

        tagCorC[0] += camera_xyz[0];
        tagCorC[1] += camera_xyz[1];
        tagCorC[2] += camera_xyz[2];

        //
        tagCorC[0] = tagCorC[0] / tagCorC[2];
        tagCorC[1] = tagCorC[1] / tagCorC[2];

        // radius^2
        T r2 = tagCorC[0] * tagCorC[0] + tagCorC[1] * tagCorC[1];


        T k1 = T(d(0, 0));
        T k2 = T(d(1, 0));
        T p1 = T(d(2, 0));
        T p2 = T(d(3, 0));
        T k3 = T(d(4, 0));

        // distort
        T xp = tagCorC[0];
        T yp = tagCorC[1];
        T xd = xp * (T(1) + r2 * (k1 + r2 * (k2 + r2 * k3))) + T(2) * p1 * xp * yp
            + p2 * (r2 + T(2) * xp * xp);
        T yd = yp * (T(1) + r2 * (k1 + r2 * (k2 + r2 * k3))) + T(2) * p2 * xp * yp
            + p1 * (r2 + T(2) * yp * yp);

        T fx = T(K(0, 0));
        T fy = T(K(1, 1));
        T cx = T(K(0, 2));
        T cy = T(K(1, 2));

        T predicted_x = T(fx) * xd + T(cx);
        T predicted_y = T(fy) * yd + T(cy);

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observation.x());
        residuals[1] = predicted_y - T(observation.y());

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Vector2d& observations,
        const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K)
    {
        return (new ceres::AutoDiffCostFunction<OpenCVReprojectionError, 2, 3, 4,
            3>( //  residuals, camPose, tagPose
            new OpenCVReprojectionError(observations, d, K)));
    }

    Eigen::Vector2d observation;

    // distortion coefficients
    Eigen::Matrix<double, 5, 1> d;
    Eigen::Matrix3d K;
};


struct TagReconstructionCostFunction
{
    TagReconstructionCostFunction(const Eigen::Vector2d& observation, double tagWidth,
        double tagHeight, const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K,
        const Eigen::Vector3d& tagCorner)
        : observation(observation)
        , tagWidth(tagWidth)
        , tagHeight(tagHeight)
        , tagCorner(tagCorner)
        , d(d)
        , K(K)
    {
    }

    template <typename T>
    bool operator()(const T* const camera_xyz, const T* const camera_q, const T* const tagPose_xyz,
        const T* const tagPose_q, T* residuals) const
    {

        // tag[1,2,3] is the marker rotation as angle axis
        T tagCor[3]{ (T)tagCorner.x(), (T)tagCorner.y(), (T)tagCorner.z() };
        T tagCorW[3];
        ceres::QuaternionRotatePoint(tagPose_q, tagCor, tagCorW);

        // translation
        tagCorW[0] += tagPose_xyz[0];
        tagCorW[1] += tagPose_xyz[1];
        tagCorW[2] += tagPose_xyz[2];

        // transform into camera coordinate system
        T tagCorC[3];
        ceres::QuaternionRotatePoint(camera_q, tagCorW, tagCorC);

        tagCorC[0] += camera_xyz[0];
        tagCorC[1] += camera_xyz[1];
        tagCorC[2] += camera_xyz[2];

        //
        tagCorC[0] = tagCorC[0] / tagCorC[2];
        tagCorC[1] = tagCorC[1] / tagCorC[2];

        // radius^2
        T r2 = tagCorC[0] * tagCorC[0] + tagCorC[1] * tagCorC[1];


        T k1 = T(d(0, 0));
        T k2 = T(d(1, 0));
        T p1 = T(d(2, 0));
        T p2 = T(d(3, 0));
        T k3 = T(d(4, 0));

        // distort
        T xp = tagCorC[0];
        T yp = tagCorC[1];
        T xd = xp * (T(1) + r2 * (k1 + r2 * (k2 + r2 * k3))) + T(2) * p1 * xp * yp
            + p2 * (r2 + T(2) * xp * xp);
        T yd = yp * (T(1) + r2 * (k1 + r2 * (k2 + r2 * k3))) + T(2) * p2 * xp * yp
            + p1 * (r2 + T(2) * yp * yp);

        T fx = T(K(0, 0));
        T fy = T(K(1, 1));
        T cx = T(K(0, 2));
        T cy = T(K(1, 2));

        T predicted_x = T(fx) * xd + T(cx);
        T predicted_y = T(fy) * yd + T(cy);

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observation.x());
        residuals[1] = predicted_y - T(observation.y());

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Eigen::Vector2d& observations, double tagWidth,
        double tagHeight, const Eigen::Matrix<double, 5, 1>& d, const Eigen::Matrix3d& K,
        const Eigen::Vector3d& tagCorner)
    {
        return (new ceres::AutoDiffCostFunction<TagReconstructionCostFunction, 2, 3, 4, 3,
            4>( //  residuals, camPose, tagPose
            new TagReconstructionCostFunction(observations, tagWidth, tagHeight, d, K, tagCorner)));
    }

    Eigen::Vector2d observation;
    double tagWidth, tagHeight;
    const Eigen::Vector3d tagCorner;

    // distortion coefficients
    Eigen::Matrix<double, 5, 1> d;
    Eigen::Matrix3d K;
};
}
#endif
