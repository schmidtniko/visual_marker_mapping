#ifndef VISUAL_MARKER_MAPPING_TAGRECONSTRUCTOR_H_
#define VISUAL_MARKER_MAPPING_TAGRECONSTRUCTOR_H_

#include "visual_marker_mapping/Camera.h"
#include "visual_marker_mapping/CameraModel.h"
#include "visual_marker_mapping/TagDetector.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <map>
#include <vector>

namespace visual_marker_mapping
{
struct ReconstructedTag
{
    int id = -1;
    std::string tagType = "";
    Eigen::Vector4d q = Eigen::Vector4d::UnitX();
    Eigen::Vector3d t = Eigen::Vector3d::Zero(); // translation vector
    double tagWidth = 0.0;
    double tagHeight = 0.0;

    Eigen::Quaterniond quat() const { return Eigen::Quaterniond(q(0), q(1), q(2), q(3)); }
    void setQuat(const Eigen::Quaterniond& quat)
    {
        q(0) = quat.w();
        q(1) = quat.x();
        q(2) = quat.y();
        q(3) = quat.z();
    }

    std::vector<Eigen::Vector3d> computeMarkerCorners3D() const
    {
        std::vector<Eigen::Vector3d> markerCorners3D = computeLocalMarkerCorners3D();

        const Eigen::Matrix3d R = quat().toRotationMatrix();

        for (size_t i = 0; i < 4; ++i)
            markerCorners3D[i] = R * markerCorners3D[i] + t;

        return markerCorners3D;
    }
    std::vector<Eigen::Vector3d> computeLocalMarkerCorners3D() const
    {
        std::vector<Eigen::Vector3d> markerCorners3D;
        markerCorners3D.push_back(Eigen::Vector3d(-tagWidth / 2.0, -tagHeight / 2.0, 0));
        markerCorners3D.push_back(Eigen::Vector3d(tagWidth / 2.0, -tagHeight / 2.0, 0));
        markerCorners3D.push_back(Eigen::Vector3d(tagWidth / 2.0, tagHeight / 2.0, 0));
        markerCorners3D.push_back(Eigen::Vector3d(-tagWidth / 2.0, tagHeight / 2.0, 0));
        return markerCorners3D;
    }
};

std::map<std::uint32_t, Eigen::Vector3d> flattenReconstruction(
    const std::map<int, ReconstructedTag>& reconstructedTags);

class TagReconstructor
{
public:
    TagReconstructor(DetectionResult detection_result);

    /**
     * Starts the reconstruction
     * @param numThreads Number of threads with which ceres is working
     */
    void startReconstruction(size_t numThreads = 1);

    /**
     * Calculates the reprojection error per image.
     */
    const std::map<int, double> computeReprojectionErrorPerImg() const;

    /**
     * Calculates the reprojection error per tag
     * @return A map wih key=tagId and value=average reprojection error for the tag with tagId
     */
    const std::map<int, double> computeReprojectionErrorPerTag(double& avg) const;

    /**
     * Calculates the reprojection error per detected corner.
     */
    const std::vector<Eigen::Vector2d> computeReprojectionErrorPerCorner() const;

    /**
     * Transforms an specific tag into the origin of the world and all other tags and cameras
     * correspondingly to this transformation.
     * @param tagId Id of the tag which should be the origin
     */
    void moveTagIntoOrigin(int tagId);

    /**
     * Returns the image id which corresponds to the image name
     * @param name Name of the image
     */
    // int imgName2ImgId(const std::string& name);

    int getLowestTag() const;

    const std::map<int, ReconstructedTag> getReconstructedTags() const;
    const std::map<int, Camera> getReconstructedCameras() const;

    /**
     * Removes reconstructed Markers with a reprojection error which is
     * higher then a threshold.
     */
    void removeBadMarkers(double threshold);

    /**
     * Removes cameras with a reprojection error which is higher then a
     * threshold.
     */
    void removeBadCameras(double threshold);

    CameraModel getCameraModel() const;
    void setCameraModel(const CameraModel& cameraModel);

    void setOriginTagId(int originTagId);

protected:
    /**
     * Calculates the relatvie camera pose for an image.
     * @param
     */
    bool computeRelativeCameraPoseFromImg(int imageId, const Eigen::Matrix3d& K,
        const Eigen::Matrix<double, 5, 1>& distCoefficients, Eigen::Quaterniond& q,
        Eigen::Vector3d& t);

    // void doBundleAdjustment_points(int maxNumIterations,
    //                         int ceresThreads, bool printSummary=false);
    void doBundleAdjustment(int maxNumIterations, size_t ceresThreads, bool robustify = true,
        bool printSummary = false);

    int originTagId;

    DetectionResult detectionResults_;

    std::map<int, ReconstructedTag> reconstructedTags; // tag id too reconstructed tag
    std::map<int, Camera> reconstructedCameras; // image id too reconstructed Camera

    /**
     * Camera model which represents the intrinsic calibration of the cameras with which the images
     * where taken.
     */
    CameraModel camModel;
};
}

#endif
