#include "visual_marker_mapping/TagReconstructor.h"
#include "visual_marker_mapping/CameraUtilities.h"
#include "visual_marker_mapping/EigenCVConversions.h"
#include "visual_marker_mapping/ReconstructionIO.h"
#include "visual_marker_mapping/TagReconstructionCostFunction.h"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/StdVector>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/rotation.h>
#include <ceres/version.h>
#include <memory>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace visual_marker_mapping
{
//-------------------------------------------------------------------------------------------------
template <typename Map1, typename Map2, typename F>
void iterateMatches(const Map1& m1, const Map2& m2, F&& f)
{
    if (m1.size() < m2.size())
    {
        for (auto it1 = std::begin(m1); it1 != std::end(m1); ++it1)
        {
            const auto it2 = m2.find(it1->first);
            if (it2 == m2.end())
                continue;
            f(it1->first, it1->second, it2->second);
        }
    }
    else
    {
        for (auto it2 = std::begin(m2); it2 != std::end(m2); ++it2)
        {
            const auto it1 = m1.find(it2->first);
            if (it1 == m1.end())
                continue;
            f(it1->first, it1->second, it2->second);
        }
    }
}
//-------------------------------------------------------------------------------------------------
std::map<std::uint32_t, Eigen::Vector3d> flattenReconstruction(
    const std::map<int, ReconstructedTag>& reconstructedTags)
{
    std::map<std::uint32_t, Eigen::Vector3d> ret;
    for (const auto& reconstTag : reconstructedTags)
    {
        const int tagId = reconstTag.second.id;
        const auto corners = reconstTag.second.computeMarkerCorners3D();

        assert(tagId >= 0);
        const std::uint32_t utagId = static_cast<std::uint32_t>(tagId);

        for (std::uint32_t i = 0; i < 4; i++)
        {
            const std::uint32_t id = (utagId << 2) + i;
            ret.emplace(id, corners[i]);
        }
    }
    return ret;
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
TagReconstructor::TagReconstructor(DetectionResult detection_result)
    : originTagId(-1)
    , detectionResults_(std::move(detection_result))
{
}
//-------------------------------------------------------------------------------------------------
int TagReconstructor::getLowestTag() const
{
    int min = detectionResults_.tags[0].tagId;
    for (const auto& tagIt : detectionResults_.tags)
    {
        if (tagIt.tagId < min)
            min = tagIt.tagId;
    }
    return min;
}
//-------------------------------------------------------------------------------------------------
void TagReconstructor::startReconstruction(size_t numThreads)
{
    if (originTagId == -1)
    {
        originTagId = getLowestTag();
    }

    const Eigen::Matrix3d K = camModel.getK();
    const Eigen::Matrix<double, 5, 1> distCoefficents = camModel.distortionCoefficients;

    // map to find out which images observe a certain tag
    std::map<int, std::set<int> > whichImagesObserveTag; // tagid -> [imageId]
    for (const auto& tagObs : detectionResults_.tagObservations)
        whichImagesObserveTag[tagObs.tagId].insert(tagObs.imageId);

    std::map<int, std::set<int> > tagsInImage; // imageId -> [tagId]
    for (const auto& tagObs : detectionResults_.tagObservations)
        tagsInImage[tagObs.imageId].insert(tagObs.tagId);

    std::map<int, std::string> imageFilenames;
    for (const auto& img : detectionResults_.images)
        imageFilenames[img.imageId] = img.filename;

    std::map<int, Tag*> tagById;
    for (auto& tag : detectionResults_.tags)
        tagById[tag.tagId] = &tag;

    // find image with the most markers inside among which the startid is
    size_t maxObservations = 0;
    int curImageId = -1;
    for (int imageId : whichImagesObserveTag[originTagId])
    {
        if (tagsInImage[imageId].size() > maxObservations)
        {
            curImageId = imageId;
            maxObservations = tagsInImage[imageId].size();
        }
    }

    if (tagById.find(originTagId) == tagById.end())
    {
        std::string errMsg = "Could not use tag with id " + std::to_string(originTagId)
            + " as origin tag, because it was not detected.";
        throw std::runtime_error(errMsg);
    }

    // set Tag with start id into the origin
    ReconstructedTag recTag;
    recTag.id = originTagId;
    recTag.tagType = tagById[originTagId]->tagType;
    recTag.t = Eigen::Vector3d::Zero();
    recTag.setQuat(Eigen::Quaterniond::Identity());
    recTag.tagWidth = tagById[originTagId]->width;
    recTag.tagHeight = tagById[originTagId]->height;
    reconstructedTags.emplace(originTagId, recTag);

    // Init with the first tag id. If no tag is reconstructed at the beginning
    // this
    // tag id will be taken to reconstruct the inital tag
    while (1)
    {
        std::cout << "Reconstructing image " << reconstructedCameras.size() << "/"
                  << detectionResults_.images.size() << " | " << imageFilenames[curImageId]
                  << " with id: " << curImageId << std::endl;

        Camera newCamera;
        newCamera.cameraId = curImageId;
        Eigen::Quaterniond q;

        if (!computeRelativeCameraPoseFromImg(curImageId, K, distCoefficents, q, newCamera.t))
        {
            throw std::runtime_error(
                "No reconstructed tags in image found. To reconstruct the image pose "
                "already reconstructed markers are needed. This should NOT happen.");
        }

        newCamera.setQuat(q);
        std::cout << "   Initialized camera with id " << curImageId << std::endl;

        reconstructedCameras.emplace(curImageId, newCamera);
        for (const auto& tagObs : detectionResults_.tagObservations)
        {
            if (tagObs.imageId != curImageId)
                continue;
            // reconstruct tag if not already reconstructed
            if (reconstructedTags.count(tagObs.tagId))
                continue;

            // check if tag is large enough in image to avoid a wrong reconstruction
            if (0)
            {
                const double widthInImage = (tagObs.corners[0] - tagObs.corners[1]).norm();
                const double heightInImage = (tagObs.corners[1] - tagObs.corners[2]).norm();
                const double tagRatio = widthInImage < heightInImage ? widthInImage / heightInImage
                                                                     : heightInImage / widthInImage;
                if (tagRatio < 0.6)
                {
                    std::cout << "   Skipping reconstruction of tag " << tagObs.tagId
                              << ": Bad aspect ratio: " << tagRatio << " < 0.6" << std::endl;
                    continue;
                }
            }
            if (whichImagesObserveTag[tagObs.tagId].size() < 2)
            {
                std::cout << "   Skipping reconstruction of tag " << tagObs.tagId
                          << ": Only observed once!" << std::endl;
                continue;
            }

            ReconstructedTag recTag;
            recTag.id = tagObs.tagId;
            recTag.tagType = tagById[tagObs.tagId]->tagType;
            recTag.tagWidth = tagById[tagObs.tagId]->width;
            recTag.tagHeight = tagById[tagObs.tagId]->height;

            // std::cout << "Adding new tag with id " << tagId << " from camera " <<
            // camId << std::endl;
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            solvePnPEigen(recTag.computeLocalMarkerCorners3D(), // could be a static fn
                tagObs.corners, K, distCoefficents, R, t);

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = R;
            T.block<3, 1>(0, 3) = t;

            // const Eigen::Matrix4d cameraTransformation =
            // reconstructedCameras[curImageId].getCameraTransformation();
            Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
            extrinsic.block<3, 3>(0, 0)
                = reconstructedCameras[curImageId].quat().toRotationMatrix();
            extrinsic.block<3, 1>(0, 3) = reconstructedCameras[curImageId].t;

            const Eigen::Matrix4d TMarker2World = extrinsic.inverse() * T;
            recTag.setQuat(Eigen::Quaterniond(TMarker2World.block<3, 3>(0, 0)));
            recTag.t = TMarker2World.block<3, 1>(0, 3);

            reconstructedTags.emplace(tagObs.tagId, recTag);

            std::cout << "   Initialized tag with id " << tagObs.tagId << std::endl;
            // std::cout << recTag.R  << "\n" << recTag.t.transpose() << std::endl;
        }

        // doBundleAdjustment_points(1500, numThreads);
        doBundleAdjustment(400, numThreads, true);

        // remove bad markers
        removeBadMarkers(2.0);

        // find next image
        int maxPairs = 0;
        for (const auto& img : detectionResults_.images)
        {
            if (reconstructedCameras.count(img.imageId))
                continue;

            // look for an unreconstructed image with the maximum number of
            // reconsturcted tags
            int numCorrespondences = 0;
            for (int tagId : tagsInImage[img.imageId])
            {
                if (reconstructedTags.count(tagId))
                    numCorrespondences++;
            }
            if (numCorrespondences > maxPairs)
            {
                maxPairs = numCorrespondences;
                curImageId = img.imageId;
            }
        }

        if (maxPairs == 0)
            break;

        // if (reconstructedCameras.size()==1)
        // break;

        std::cout << "----------------------------------------------------------" << std::endl;
    }

    std::cout << "Starting final bundle adjustment" << std::endl;

    doBundleAdjustment(1500, numThreads, true, false);

    removeBadMarkers(2.0);

    removeBadCameras(2.0);

    doBundleAdjustment(1500, numThreads, false, true);
}
//-----------------------------------------------------------------------------
bool TagReconstructor::computeRelativeCameraPoseFromImg(int imageId, const Eigen::Matrix3d& K,
    const Eigen::Matrix<double, 5, 1>& distCoefficients, Eigen::Quaterniond& q, Eigen::Vector3d& t)
{
    std::vector<Eigen::Vector3d> markerCorners3D;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > observations2D;
    // find all matches between this image and the reconstructions
    for (const auto& tagObs : detectionResults_.tagObservations)
    {
        if (tagObs.imageId != imageId)
            continue;
        const auto tagIt = reconstructedTags.find(tagObs.tagId);
        if (tagIt == reconstructedTags.end())
            continue;

        const std::vector<Eigen::Vector3d> tagCorners = tagIt->second.computeMarkerCorners3D();

        markerCorners3D.insert(markerCorners3D.begin(), tagCorners.begin(), tagCorners.end());
        observations2D.insert(observations2D.begin(), tagObs.corners.begin(), tagObs.corners.end());
    }
    std::cout << "   Reconstructing camera pose from " << observations2D.size()
              << " 2d/3d correspondences" << std::endl;

    if (observations2D.empty())
        return false;

    Eigen::Matrix3d R;
    // solvePnPEigen(markerCorners3D, observations2D, K, distCoefficients, R, t);
    solvePnPRansacEigen(markerCorners3D, observations2D, K, distCoefficients, R, t);

    q = Eigen::Quaterniond(R);

    return true;
}
//-----------------------------------------------------------------------------
void TagReconstructor::moveTagIntoOrigin(int tagId)
{
    if (!reconstructedTags.count(tagId))
        throw std::runtime_error("Tag with id " + std::to_string(tagId) + " is not reconstructed.");

    std::cout << "Transforming tag with id " << tagId << " into origin." << std::endl;

    const Eigen::Quaterniond q = reconstructedTags[tagId].quat().inverse();
    const Eigen::Vector3d t = reconstructedTags[tagId].t;

    for (auto& tag : reconstructedTags)
    {
        tag.second.setQuat(q * tag.second.quat());
        tag.second.t = q.toRotationMatrix() * (tag.second.t - t);
    }

    for (auto& camPair : reconstructedCameras)
    {
        const Camera& cam = camPair.second;
        camPair.second.setQuat(q * cam.quat());
        camPair.second.t = q.toRotationMatrix() * (cam.t - t);
    }

    std::cout << "Finished transforming Tags" << std::endl;
}
//-----------------------------------------------------------------------------
const std::map<int, double> TagReconstructor::computeReprojectionErrorPerImg() const
{
    std::map<int, double> repErrorMap;
    std::map<int, int> numObsMap;
    for (const auto& tagObs : detectionResults_.tagObservations)
    {
        const auto camIt = reconstructedCameras.find(tagObs.imageId);
        if (camIt == reconstructedCameras.end())
            continue;
        const auto tagIt = reconstructedTags.find(tagObs.tagId);
        if (tagIt == reconstructedTags.end())
            continue;

        const auto pts3d = tagIt->second.computeMarkerCorners3D();
        const auto& camera = camIt->second;

        const Eigen::Matrix3d R = camera.quat().toRotationMatrix();
        const Eigen::Vector3d t = camera.t;
        double sum = 0.0;
        for (size_t i = 0; i < pts3d.size(); ++i)
        {
            const Eigen::Vector2d dist
                = camModel.projectPoint(R * pts3d[i] + t) - tagObs.corners[i];
            sum += dist.norm();
        }

        numObsMap[tagObs.imageId] += pts3d.size();
        repErrorMap[tagObs.imageId] += sum;
    }

    int numTotal = 0;
    for (auto& errs : repErrorMap)
    {
        errs.second /= numObsMap[errs.first];
        numTotal += numObsMap[errs.first];
    }

    // set the reprojection error of cameras without any observations to something
    // invalid
    for (const auto& repCam : reconstructedCameras)
    {
        if (repErrorMap.count(repCam.first) == 0)
            repErrorMap[repCam.first] = -1.0;
    }
    return repErrorMap;
}
//-----------------------------------------------------------------------------
const std::map<int, double> TagReconstructor::computeReprojectionErrorPerTag(double& avg) const
{
    std::map<int, double> repErrorMap;
    std::map<int, int> numObsMap;
    for (const auto& tagObs : detectionResults_.tagObservations)
    {
        const auto camIt = reconstructedCameras.find(tagObs.imageId);
        if (camIt == reconstructedCameras.end())
            continue;
        const auto tagIt = reconstructedTags.find(tagObs.tagId);
        if (tagIt == reconstructedTags.end())
            continue;

        const auto pts3d = tagIt->second.computeMarkerCorners3D();
        const auto& camera = camIt->second;

        const Eigen::Matrix3d R = camera.quat().toRotationMatrix();
        const Eigen::Vector3d t = camera.t;
        double sum = 0.0;
        for (size_t i = 0; i < pts3d.size(); ++i)
        {
            const Eigen::Vector2d dist
                = camModel.projectPoint(R * pts3d[i] + t) - tagObs.corners[i];
            sum += dist.norm();
        }
        numObsMap[tagObs.tagId] += pts3d.size();
        repErrorMap[tagObs.tagId] += sum;
    }

    avg = 0;
    int numTotal = 0;
    for (auto& errs : repErrorMap)
    {
        avg += errs.second;
        errs.second /= numObsMap[errs.first];
        numTotal += numObsMap[errs.first];
    }

    if (numTotal)
        avg /= numTotal;
    return repErrorMap;
}
//-----------------------------------------------------------------------------
const std::vector<Eigen::Vector2d> TagReconstructor::computeReprojectionErrorPerCorner() const
{
    std::vector<Eigen::Vector2d> reprojectionErrors;
    for (const auto& tagObs : detectionResults_.tagObservations)
    {
        const auto camIt = reconstructedCameras.find(tagObs.imageId);
        if (camIt == reconstructedCameras.end())
            continue;
        const auto tagIt = reconstructedTags.find(tagObs.tagId);
        if (tagIt == reconstructedTags.end())
            continue;

        const auto pts3d = tagIt->second.computeMarkerCorners3D();
        const auto& camera = camIt->second;

        const Eigen::Matrix3d R = camera.quat().toRotationMatrix();
        const Eigen::Vector3d t = camera.t;
        for (size_t i = 0; i < pts3d.size(); ++i)
        {
            reprojectionErrors.push_back(
                camModel.projectPoint(R * pts3d[i] + t) - tagObs.corners[i]);
        }
    }

    return reprojectionErrors;
}
//-----------------------------------------------------------------------------
#if 0
void TagReconstructor::doBundleAdjustment_points(int maxNumIterations,
                                int ceresThreads, bool printSummary)
{
     ceres::Problem markerBAProblem;

    std::map<int, double*> tagPosesCeres;
    std::map<int, double*> camerasCeres;

    auto ordering = new ceres::ParameterBlockOrdering;

    for (const auto& reconstTag : reconstructedTags)
    {
        int tagId = reconstTag.second.id;
        if(tagPosesCeres.count(tagId) == 0)
        {
            // Eigen::AngleAxisd aAx(reconstTag.second.R);
            // tagPosesCeres[tagId] = new double[6];
            // tagPosesCeres[tagId][0] = aAx.axis().x() * aAx.angle();
            // tagPosesCeres[tagId][1] = aAx.axis().y() * aAx.angle();
            // tagPosesCeres[tagId][2] = aAx.axis().z() * aAx.angle();
            // tagPosesCeres[tagId][3] = reconstTag.second.t.x();
            // tagPosesCeres[tagId][4] = reconstTag.second.t.y();
            // tagPosesCeres[tagId][5] = reconstTag.second.t.z();
            //Eigen::AngleAxisd aAx(reconstTag.second.R);

            auto worldCorners=reconstTag.second.computeMarkerCorners3D();
            
            tagPosesCeres[tagId] = new double[12];
            for (int i=0;i<4;i++)
            {
                tagPosesCeres[tagId][i*3+0] = worldCorners[i].x();
                tagPosesCeres[tagId][i*3+1] = worldCorners[i].y();
                tagPosesCeres[tagId][i*3+2] = worldCorners[i].z();
                markerBAProblem.AddParameterBlock(&tagPosesCeres[tagId][i*3+0], 3);
                ordering->AddElementToGroup(&tagPosesCeres[tagId][i*3+0],0);

                if ((tagId==startTagId))
                {
                    markerBAProblem.SetParameterBlockConstant(&tagPosesCeres[tagId][i*3+0]);
                }
            }

            //std::cout << "TAG : " << tagId << std::endl;
        }

        for (const auto& imageName : tagId2ImgNames[tagId]) // get list of images containing tagId
        {
            const int camId = tagImgName2tagImg[imageName].imageId;
            if(!reconstructedCameraIds.count(tagImgName2tagImg[imageName].imageId))
                continue;

            if(camerasCeres.count(camId) == 0)
            {
                Eigen::Matrix4d extrinsicMatrix = reconstructedCameras[camId].getExtrinsicMatrix();
                Eigen::Matrix3d R = extrinsicMatrix.block<3,3>(0,0);
                //Eigen::AngleAxisd aAx(R);

                Eigen::Quaterniond q;
                q=R;

                Eigen::Vector3d t = extrinsicMatrix.block<3,1>(0,3);

                camerasCeres[camId] = new double[7];
                camerasCeres[camId][0] = q.w();
                camerasCeres[camId][1] = q.x();
                camerasCeres[camId][2] = q.y();
                camerasCeres[camId][3] = q.z();
                camerasCeres[camId][4] = t.x();
                camerasCeres[camId][5] = t.y();
                camerasCeres[camId][6] = t.z();

                ceres::LocalParameterization* quaternion_parameterization = new ceres::QuaternionParameterization;
                markerBAProblem.AddParameterBlock(&camerasCeres[camId][0], 4, quaternion_parameterization);

                markerBAProblem.AddParameterBlock(&camerasCeres[camId][0]+4, 3);

                ordering->AddElementToGroup(&camerasCeres[camId][0],1);
                ordering->AddElementToGroup(&camerasCeres[camId][0]+4,1);
            }

            Eigen::Matrix<double, 5, 1> d = reconstructedCameras[camId].getDistcoefficents();
            Eigen::Matrix3d K = reconstructedCameras[camId].getCalibrationMatrix().block<3,3>(0,0);

//            std::cout << "d =" << d << std::endl;
//            std::cout << "K = " << K << std::endl;
//            std::cout << "tagImgName2TagImg[imageName].observations[tagId].corners.size() = " << tagImgName2TagImg[imageName].observations[tagId].corners.size() << std::endl;
            //------

            auto& actTagImage = tagImgName2tagImg[imageName];
            auto& actObservations = actTagImage.observations[tagId];

            for (int i = 0; i < 4; i++)
            {
                auto* currentCostFunc = OpenCVReprojectionError::Create(actObservations.corners[i],
                        d,
                        K);

                markerBAProblem.AddResidualBlock(currentCostFunc,
                                                 nullptr,//new ceres::HuberLoss(1.0),//nullptr,//new ceres::CauchyLoss(3),
                                                 camerasCeres[camId]+4,
                                                 camerasCeres[camId]+0,
                                                 &tagPosesCeres[tagId][i*3+0]);
            }
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_ordering.reset(ordering);

    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = maxNumIterations;
    options.num_threads=ceresThreads;
    options.num_linear_solver_threads=ceresThreads;
    //options.gradient_tolerance = 1e-10;
    //options.function_tolerance = 1e-10;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    //options.
    options.eta = 1e-2;

    ceres::Solver::Summary summary;
    Solve(options, &markerBAProblem, &summary);
    std::cout << "Solution " << summary.termination_type << std::endl;
    //if (printSummary)
        std::cout << summary.FullReport() << std::endl;

    for (const auto& cam : camerasCeres)
    {
        Eigen::Quaterniond q;
        q.w()=cam.second[0];
        q.x()=cam.second[1];
        q.y()=cam.second[2];
        q.z()=cam.second[3];
        Eigen::Matrix3d R=q.toRotationMatrix();

        Eigen::Vector3d t;
        t.x() = cam.second[4];
        t.y() = cam.second[5];
        t.z() = cam.second[6];

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;

        //std::cout << "Changing camera extrinsic from : \n" << reconstructedCameras[cam.first].getExtrinsicMatrix() << "to: \n" << T << std::endl;

        reconstructedCameras[cam.first].setExtrinsicMatrix(T);
        delete[] camerasCeres[cam.first];
    }

    bool b=false;
    for (const auto& tag : tagPosesCeres)
    {
        Eigen::Vector3d v1(tag.second[0],tag.second[1],tag.second[2]); // ul
        Eigen::Vector3d v2(tag.second[3],tag.second[4],tag.second[5]); // ur
        Eigen::Vector3d v3(tag.second[6],tag.second[7],tag.second[8]); // or
        Eigen::Vector3d v4(tag.second[9],tag.second[10],tag.second[11]); // ol

        if (!b)
        {
            std::cout << v1 << std::endl;
            b=true;
        }

        Eigen::Vector3d x = (v2-v1).normalized();
        Eigen::Vector3d y = (v4-v1).normalized();
        Eigen::Vector3d z = x.cross(y).normalized();;

        Eigen::Matrix3d R;
        R.col(0)=y;
        R.col(1)=-x;
        R.col(2)=z;
        //R(2,1)=-R(2,1);
        //std::swap(R(2,0),R(2,1));


        Eigen::Vector3d t=(v1+v2+v3+v4)/4;

        //std::cout << "Changing tag extrinsic from : \n" << reconstructedTags[tag.first].R << "\nto: \n" << R << std::endl;
        //std::cout << "Changing tag extrinsicT from : \n" << reconstructedTags[tag.first].t << "\nto: \n" << t << std::endl;

        reconstructedTags[tag.first].R = R;
        reconstructedTags[tag.first].t = t;

        delete[] tagPosesCeres[tag.first];
    }
}
#endif
//-----------------------------------------------------------------------------
void TagReconstructor::doBundleAdjustment(
    int maxNumIterations, size_t ceresThreads, bool robustify, bool printSummary)
{
    std::map<int, std::set<int> > whichImagesObserveTag; // tagid -> [imageId]
    for (const auto& tagObs : detectionResults_.tagObservations)
        whichImagesObserveTag[tagObs.tagId].insert(tagObs.imageId);

    std::map<int, Tag*> tagById;
    for (auto& tag : detectionResults_.tags)
        tagById[tag.tagId] = &tag;

    ceres::Problem markerBAProblem;

    auto ordering = new ceres::ParameterBlockOrdering;

    auto quaternion_parameterization = new ceres::QuaternionParameterization;

    for (auto& reconstTag : reconstructedTags)
    {
        markerBAProblem.AddParameterBlock(&reconstTag.second.q(0), 4, quaternion_parameterization);
        markerBAProblem.AddParameterBlock(&reconstTag.second.t(0), 3);

        const int tagId = reconstTag.second.id;
        if (tagId == originTagId)
        {
            markerBAProblem.SetParameterBlockConstant(&reconstTag.second.q(0));
            markerBAProblem.SetParameterBlockConstant(&reconstTag.second.t(0));
        }

        ordering->AddElementToGroup(&reconstTag.second.q(0), 0);
        ordering->AddElementToGroup(&reconstTag.second.t(0), 0);
    }

    std::map<int, int> numTagsInImage;
    for (const auto& tagObs : detectionResults_.tagObservations)
    {
        if (reconstructedTags.count(tagObs.tagId))
            numTagsInImage[tagObs.imageId]++;
    }

    for (auto& recCam : reconstructedCameras) // get list of images containing tagId
    {
        const int imageId = recCam.first;
        if (!numTagsInImage[imageId])
            continue;

        markerBAProblem.AddParameterBlock(&recCam.second.q(0), 4, quaternion_parameterization);
        markerBAProblem.AddParameterBlock(&recCam.second.t(0), 3);

        ordering->AddElementToGroup(&recCam.second.q(0), 1);
        ordering->AddElementToGroup(&recCam.second.t(0), 1);
    }

    for (const auto& tagObs : detectionResults_.tagObservations)
    {
        const auto camIt = reconstructedCameras.find(tagObs.imageId);
        if (camIt == reconstructedCameras.end())
            continue;
        auto& recCam = camIt->second;
        const auto tagIt = reconstructedTags.find(tagObs.tagId);
        if (tagIt == reconstructedTags.end())
            continue;
        auto& recTag = tagIt->second;

        const Eigen::Matrix<double, 5, 1> d = camModel.distortionCoefficients;
        const Eigen::Matrix3d K = camModel.getK();

        auto corners = reconstructedTags[tagObs.tagId].computeLocalMarkerCorners3D();

        for (size_t i = 0; i < 4; i++)
        {
            auto* currentCostFunc = TagReconstructionCostFunction::Create(tagObs.corners[i],
                tagById[tagObs.tagId]->width, tagById[tagObs.tagId]->height, d, K, corners[i]);

            markerBAProblem.AddResidualBlock(currentCostFunc,
                robustify ? new ceres::HuberLoss(1.0) : nullptr, // new ceres::CauchyLoss(3),
                &recCam.t(0), &recCam.q(0), &recTag.t(0), &recTag.q(0));
        }
    }
    ceres::Solver::Options options;
#if (CERES_VERSION_MAJOR == 1) && (CERES_VERSION_MINOR >= 11)
    options.linear_solver_ordering.reset(ordering);
#else
    options.linear_solver_ordering = ordering;
#endif
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = maxNumIterations;
    options.num_threads = static_cast<int>(ceresThreads);
    options.num_linear_solver_threads = static_cast<int>(ceresThreads);
    // options.eta = 1e-2;

    ceres::Solver::Summary summary;
    Solve(options, &markerBAProblem, &summary);

    std::cout << "Solution " << summary.termination_type << std::endl;
    if (printSummary)
        std::cout << summary.FullReport() << std::endl;

#if 1 // Compute covariances
    if (printSummary)
    {
        std::vector<std::pair<const double*, const double*> > covarianceBlocks;
        for (const auto& reconstTag : reconstructedTags)
        {
            covarianceBlocks.emplace_back(&reconstTag.second.t(0), &reconstTag.second.t(0));
        }

        ceres::Covariance::Options covOptions;
        covOptions.num_threads = static_cast<int>(ceresThreads);

        ceres::Covariance covarianceComputation(covOptions);
        covarianceComputation.Compute(covarianceBlocks, &markerBAProblem);

        std::map<int, Eigen::Matrix3d> covariances;
        Eigen::Vector3d averageCovDiagonal = Eigen::Vector3d::Zero();
        for (const auto& reconstTag : reconstructedTags)
        {
            const int tagId = reconstTag.second.id;
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covariance;
            covarianceComputation.GetCovarianceBlock(
                &reconstTag.second.t(0), &reconstTag.second.t(0), &covariance(0, 0));
            covariances.emplace(tagId, covariance);

            const Eigen::Vector3d diag = covariance.diagonal();
            Eigen::Vector3d std_diag;
            std_diag << sqrt(diag(0)), sqrt(diag(1)), sqrt(diag(2));
            std::cout << "StdDev of tag " << tagId << ": " << std_diag.transpose()
                      << " | StdDevNorm: " << sqrt(std_diag.norm()) << std::endl;

            averageCovDiagonal += diag;
        }
        averageCovDiagonal /= reconstructedTags.size();
        Eigen::Vector3d averageStdDevDiagonal;
        averageStdDevDiagonal << sqrt(averageCovDiagonal(0)), sqrt(averageCovDiagonal(1)),
            sqrt(averageCovDiagonal(2));
        std::cout << "Marker Position RMS = " << averageStdDevDiagonal.norm() << std::endl;
    }
#endif
}
//-----------------------------------------------------------------------------
void TagReconstructor::removeBadMarkers(double threshold)
{
    double avgRepError = 0;
    const auto reperrors = computeReprojectionErrorPerTag(avgRepError);
    for (const auto& tr : reperrors)
    {
        if (tr.second > threshold)
        {
            if (tr.first != originTagId)
            {
                std::cout << "Removing bad marker with id " << tr.first
                          << " and reprojection error " << tr.second << std::endl;
                reconstructedTags.erase(tr.first);
            }
        }
    }
}
//-----------------------------------------------------------------------------
void TagReconstructor::removeBadCameras(double threshold)
{
    const auto repErrors = computeReprojectionErrorPerImg();
    for (const auto& tr : repErrors)
    {
        if ((tr.second > threshold) || (tr.second < 0))
        {
            std::cout << "Removing bad camera with id " << tr.first << " and reprojection error "
                      << tr.second << std::endl;
            reconstructedCameras.erase(tr.first);
        }
    }
}
//-----------------------------------------------------------------------------
const std::map<int, visual_marker_mapping::ReconstructedTag>
TagReconstructor::getReconstructedTags() const
{
    return reconstructedTags;
}
//-----------------------------------------------------------------------------
const std::map<int, visual_marker_mapping::Camera> TagReconstructor::getReconstructedCameras() const
{
    return reconstructedCameras;
}
//-----------------------------------------------------------------------------
void TagReconstructor::setCameraModel(const CameraModel& cameraModel)
{
    camModel = cameraModel;
}
//-----------------------------------------------------------------------------
CameraModel TagReconstructor::getCameraModel() const
{
    return camModel;
}
//-----------------------------------------------------------------------------
void TagReconstructor::setOriginTagId(int originTagId)
{
    this->originTagId = originTagId;
}
}
