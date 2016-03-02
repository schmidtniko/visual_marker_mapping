#include "visual_marker_mapping/CameraModel.h"
#include <iostream>

namespace camSurv
{
Eigen::Vector2d CameraModel::projectPoint(const Eigen::Vector3d& point3D) const
{
    // transformation into the image plane
    Eigen::Vector3d pt = point3D / point3D.z();

    const double& k1 = distortionCoefficients(0, 0);
    const double& k2 = distortionCoefficients(1, 0);
    const double& k3 = distortionCoefficients(4, 0);

    const double& p1 = distortionCoefficients(2, 0);
    const double& p2 = distortionCoefficients(3, 0);

    const double r2 = pt.x() * pt.x() + pt.y() * pt.y();

    pt.x() = pt.x() * (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) + 2 * p1 * pt.x() * pt.y()
        + p2 * (r2 + 2 * pt.x() * pt.x());
    pt.y() = pt.y() * (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) + 2 * p2 * pt.x() * pt.y()
        + p1 * (r2 + 2 * pt.y() * pt.y());

    return (getK() * pt).head(2);
}

Eigen::Matrix3d CameraModel::getK() const
{
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = fx;
    K(1, 1) = fy;
    K(0, 2) = cx;
    K(1, 2) = cy;
    return K;
}
}
