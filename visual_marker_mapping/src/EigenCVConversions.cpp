#include "visual_marker_mapping/EigenCVConversions.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace visual_marker_mapping
{
//-----------------------------------------------------------------------------
void eigen2cv(const Eigen::Vector3d& vecEig, cv::Point3f& ptCv)
{
    ptCv.x = static_cast<float>(vecEig.x());
    ptCv.y = static_cast<float>(vecEig.y());
    ptCv.z = static_cast<float>(vecEig.z());
}
//-----------------------------------------------------------------------------
void cv2eigen(const cv::Point2f& ptCv, Eigen::Vector2d& vecEig)
{
    vecEig.x() = ptCv.x;
    vecEig.y() = ptCv.y;
}
//-----------------------------------------------------------------------------
void eigen2cv(
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& vecEigen,
    std::vector<cv::Point2d>& vecCV)
{
    vecCV.resize(vecEigen.size());
    for (size_t i = 0; i < vecCV.size(); ++i)
        vecCV[i] = cv::Point2d(vecEigen[i].x(), vecEigen[i].y());
}
//-----------------------------------------------------------------------------
void eigen2cv(const std::vector<Eigen::Vector3d>& vecEigen, std::vector<cv::Point3d>& vecCV)
{
    vecCV.resize(vecEigen.size());
    for (size_t i = 0; i < vecCV.size(); ++i)
        vecCV[i] = cv::Point3d(vecEigen[i].x(), vecEigen[i].y(), vecEigen[i].z());
}
//-----------------------------------------------------------------------------
void solvePnPEigen(const std::vector<Eigen::Vector3d>& objectPoints,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& observations,
    const Eigen::Matrix3d& K, const Eigen::Matrix<double, 5, 1>& distCoefficents,
    Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    std::vector<cv::Point3d> objectPointsCv;
    eigen2cv(objectPoints, objectPointsCv);

    std::vector<cv::Point2d> observationsCv;
    eigen2cv(observations, observationsCv);

    cv::Mat distCoefficentsCv(static_cast<int>(distCoefficents.rows()), 1, CV_64FC1);
    for (int i = 0; i < distCoefficents.rows(); ++i)
        distCoefficentsCv.at<double>(i, 0) = distCoefficents(i, 0);

    cv::Mat KCv;
    cv::eigen2cv(K, KCv);

    cv::Mat rCv, tCv;
    cv::solvePnP(
        objectPointsCv, observationsCv, KCv, distCoefficentsCv, rCv, tCv, false, CV_ITERATIVE);

    cv::Mat RCv;
    cv::Rodrigues(rCv, RCv);
    cv::cv2eigen(RCv, R);

    cv::cv2eigen(tCv, t);
}
//-----------------------------------------------------------------------------
void solvePnPRansacEigen(const std::vector<Eigen::Vector3d>& objectPoints,
    const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& observations,
    const Eigen::Matrix3d& K, const Eigen::Matrix<double, 5, 1>& distCoefficents,
    Eigen::Matrix3d& R, Eigen::Vector3d& t)
{

    if (objectPoints.size() != observations.size())
    {
        std::string error = "For solvePnPRansac the same number of objectPoints "
                            "and observations is needed. ";
        error += "Num objectPoints: " + std::to_string(objectPoints.size());
        error += " Num observations: " + std::to_string(observations.size());
        throw std::runtime_error(error);
    }

    std::vector<cv::Point3f> objectPointsCv;
    for (const auto& p : objectPoints)
        objectPointsCv.emplace_back(p.x(), p.y(), p.z());

    std::vector<cv::Point2f> observationsCv;
    for (const auto& p : observations)
        observationsCv.emplace_back(p.x(), p.y());

    cv::Mat distCoefficentsCv(static_cast<int>(distCoefficents.rows()), 1, CV_64FC1);
    for (int i = 0; i < distCoefficents.rows(); ++i)
        distCoefficentsCv.at<double>(i, 0) = distCoefficents(i, 0);

    cv::Mat KCv;
    cv::eigen2cv(K, KCv);

    cv::Mat rCv, tCv;
    cv::solvePnPRansac(objectPointsCv, observationsCv, KCv, distCoefficentsCv, rCv, tCv, false); //,
    // CV_ITERATIVE);

    cv::Mat RCv;
    cv::Rodrigues(rCv, RCv);
    cv::cv2eigen(RCv, R);

    cv::cv2eigen(tCv, t);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
}
