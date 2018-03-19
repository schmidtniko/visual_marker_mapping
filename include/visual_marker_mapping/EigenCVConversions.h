#ifndef VISUAL_MARKER_MAPPING_EIGENCVCONVERSIONS_H_
#define VISUAL_MARKER_MAPPING_EIGENCVCONVERSIONS_H_

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace visual_marker_mapping
{

void eigen2cv(const Eigen::Vector3d& vecEig, cv::Point3f& ptCv);

void cv2eigen(const cv::Point2f& ptCv, Eigen::Vector2d& vecEig);

void eigen2cv(
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& vecEigen,
    std::vector<cv::Point2d>& vecCV);

void eigen2cv(const std::vector<Eigen::Vector3d>& vecEigen, std::vector<cv::Point3d>& vecCV);

void solvePnPEigen(const std::vector<Eigen::Vector3d>& objectPoints,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& observations,
    const Eigen::Matrix3d& K, const Eigen::Matrix<double, 5, 1>& distCoefficents,
    Eigen::Matrix3d& R, Eigen::Vector3d& t);

void solvePnPRansacEigen(const std::vector<Eigen::Vector3d>& objectPoints,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& observations,
    const Eigen::Matrix3d& K, const Eigen::Matrix<double, 5, 1>& distCoefficents,
    Eigen::Matrix3d& R, Eigen::Vector3d& t);
} /* namespace libba */

#endif /* EIGENCVCONVERSIONS_H_ */
