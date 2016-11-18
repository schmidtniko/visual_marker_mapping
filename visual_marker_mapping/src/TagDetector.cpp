#include "visual_marker_mapping/TagDetector.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag16h6.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"
#include "visual_marker_mapping/FileUtilities.h"
#include <set>
#include <memory>
#include <regex>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

namespace visual_marker_mapping
{
//-------------------------------------------------------------------------------------------------
DetectionResult detectTags(const std::vector<std::string>& filePaths, double markerWidth,
    double markerHeight, const std::string& tagType, bool doCornerRefinement)
{
    if (filePaths.empty()) throw std::runtime_error("Filepaths for tag detections are empty.");

    cv::Mat img = cv::imread(filePaths[0], CV_LOAD_IMAGE_GRAYSCALE);
    const int imgWidth = img.cols;
    const int imgHeight = img.rows;


    std::unique_ptr<AprilTags::TagDetector> tagDetector;
	if (tagType == "apriltag_16h5")
        tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes16h5));
	else if (tagType == "apriltag_16h6")
        tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes16h6));
    else if (tagType == "apriltag_25h7")
        tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes25h7));
    else if (tagType == "apriltag_25h9")
        tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes25h9));
    else if (tagType == "apriltag_36h9")
        tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes36h9));
    else if (tagType == "apriltag_36h11")
        tagDetector.reset(new AprilTags::TagDetector(AprilTags::tagCodes36h11));
    else
        throw std::runtime_error("Unsupported marker type " + tagType + "!");

    DetectionResult result;

    int imageId = 0;
    int filteredImageId = 0;
    std::set<int> tagIds;
    for (const auto& filePath : filePaths)
    {
        boost::filesystem::path p(filePath);

        if (!boost::filesystem::exists(p))
        {
            std::cerr << "File: " << filePath << " does not exist" << std::endl;
            continue;
        }

        std::cout << "Processing file " << (imageId + 1) << "/" << filePaths.size() << " "
                  << filePath << std::endl;

        const cv::Mat img = cv::imread(filePath, CV_LOAD_IMAGE_GRAYSCALE);

        if (img.cols != imgWidth || img.rows != imgHeight)
        {
            std::cerr << "Image " << p.filename() << " has not the correct size of " << imgHeight
                      << " x " << imgWidth << std::endl;
            continue;
        }

        cv::Mat visualization = img.clone();
        cv::cvtColor(visualization, visualization, CV_GRAY2BGR);

        const std::vector<AprilTags::TagDetection> detectedTags = tagDetector->extractTags(img);

        std::set<int> goodObservations;

        for (const auto& detectedTag : detectedTags)
        {
            if (!detectedTag.good) continue;

            goodObservations.insert(detectedTag.id);

            TagObservation tagObs;
            tagObs.imageId = filteredImageId;
            tagObs.tagId = detectedTag.id;
            tagObs.corners.resize(4);
            for (size_t i = 0; i < 4; ++i)
                tagObs.corners[i] << detectedTag.p[i].first, detectedTag.p[i].second;

            // do opencv corner refinement
            if (doCornerRefinement)
            {
                std::vector<cv::Point2f> corners;
                for (size_t i = 0; i < 4; i++)
                    corners.emplace_back(tagObs.corners[i].x(), tagObs.corners[i].y());

                cv::Size winSize = cv::Size(10, 10);
                cv::Size zeroZone = cv::Size(-1, -1);
                cv::TermCriteria criteria
                    = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001);

                /// Calculate the refined corner locations
                cv::cornerSubPix(img, corners, winSize, zeroZone, criteria);

                for (size_t i = 0; i < 4; i++)
                {
                    // std::cout << "BEFOR: " << tagObs.corners[i].transpose() <<
                    // std::endl;
                    tagObs.corners[i] << corners[i].x, corners[i].y;
                    // std::cout << "AFTER: " << tagObs.corners[i].transpose() <<
                    // std::endl;
                }
            }

            result.tagObservations.push_back(tagObs);

            tagIds.insert(detectedTag.id);
        }

        if (!goodObservations.empty())
        {
            std::cout << "   Detected " << goodObservations.size() << " tags: ";
            for (int i : goodObservations)
                std::cout << i << ", ";
            std::cout << std::endl;

            TagImg img;
            img.filename = boost::filesystem::path(filePath).string();
            img.imageId = filteredImageId;
            result.images.push_back(img);
            filteredImageId++;
        }
        else
            std::cout << "   No tags found!";

        imageId++;
    }

    for (int i : tagIds)
    {
        Tag tag;
        tag.tagId = i;
        tag.tagType = tagType;
        tag.width = markerWidth;
        tag.height = markerHeight;
        result.tags.push_back(tag);
    }

    return result;
}
//-------------------------------------------------------------------------------------------------
DetectionResult detectTags(const std::string& folder, double markerWidth, double markerHeight,
    const std::string& tagType, bool doCornerRefinement)
{
    std::regex reg(
        "(.*)\\.((png)|(jpg))", std::regex_constants::ECMAScript | std::regex_constants::icase);
    const std::vector<std::string> filePaths = readFilesFromDir(folder, reg);
    return detectTags(filePaths, markerWidth, markerHeight, tagType, doCornerRefinement);
}
//-------------------------------------------------------------------------------------------------
void visualizeDetectionResults(
    const DetectionResult& detectionResult, const std::string& exportFolder)
{
    if (!boost::filesystem::exists(exportFolder))
    {
        boost::filesystem::create_directories(exportFolder);
    }

    const cv::Scalar colorMap[4] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0) };
    for (const auto& image : detectionResult.images)
    {
        cv::Mat cvImg = cv::imread(image.filename);
        for (const auto& tagDetection : detectionResult.tagObservations)
        {
            if (tagDetection.imageId != image.imageId) continue;

            const int px
                = static_cast<int>((tagDetection.corners[0].x() + tagDetection.corners[2].x()) / 2);
            const int py
                = static_cast<int>((tagDetection.corners[0].y() + tagDetection.corners[2].y()) / 2);
            const cv::Point pos(px, py);

            cv::putText(cvImg, std::to_string(tagDetection.tagId), pos, cv::FONT_HERSHEY_SIMPLEX,
                3.5, cv::Scalar(255, 100, 0), 4);

            for (size_t i = 0; i < 4; ++i)
            {
                const cv::Scalar color = colorMap[i];
                cv::circle(cvImg,
                    cv::Point2f(tagDetection.corners[i].x(), tagDetection.corners[i].y()), 3, color,
                    3);
            }
        }

        const boost::filesystem::path p(image.filename);
        const std::string exportFilePath
            = (boost::filesystem::path(exportFolder) / p.filename()).string();
        cv::imwrite(exportFilePath, cvImg);
    }
}
//-------------------------------------------------------------------------------------------------
}
