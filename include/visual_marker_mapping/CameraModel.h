#ifndef VISUAL_MARKER_MAPPING_CAMERAMODEL_H_
#define VISUAL_MARKER_MAPPING_CAMERAMODEL_H_

#include <Eigen/Core>

namespace visual_marker_mapping
{
//! This is the calibration model that OpenCV uses.
//! X axis = right
//! Y axis = down
//! Z axis = viewing direction
struct CameraModel
{
    double fx;
    double fy;
    double cx;
    double cy;

    Eigen::Matrix<double, 5, 1> distortionCoefficients;

    int verticalResolution;
    int horizontalResolution;

    /**
     * Computes a calibration matrix K from fx, fy, cx, cy
     */
    Eigen::Matrix3d getK() const;

    /**
     * Transforms a point in camera coordinates into pixel coordinates.
     * @param point A point in camera coordinates.
     */
    Eigen::Vector2d projectPoint(const Eigen::Vector3d& point3D) const;
};
}

#endif
