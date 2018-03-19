#ifndef VISUAL_MARKER_MAPPING_CAMERA_H
#define VISUAL_MARKER_MAPPING_CAMERA_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace visual_marker_mapping
{
struct Camera
{
    int cameraId = -1;

    Eigen::Vector4d q = Eigen::Vector4d::UnitX();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    Eigen::Quaterniond quat() const { return Eigen::Quaterniond(q(0), q(1), q(2), q(3)); }
    void setQuat(const Eigen::Quaterniond& quat) { q << quat.w(), quat.x(), quat.y(), quat.z(); }
};
}

#endif
