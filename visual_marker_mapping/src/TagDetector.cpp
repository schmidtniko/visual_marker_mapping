#include "visual_marker_mapping/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "visual_marker_mapping/fileUtilities.h"
#include "visual_marker_mapping/DetectionIO.h"
#include <set>
#include <memory>
#include <map>
#include <regex>
#include <cv.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

namespace camSurv
{
TagDetector::TagDetector(const std::string& rootPath, const std::string& imgPath,
    const std::string& detectedTagsImgPath, int visHeight, int visWidth, double markerWidth,
    double markerHeight)
    : _rootPath(rootPath)
    , _imgPath(imgPath)
    , _detectedTagsImgPath(detectedTagsImgPath)
    , _visWidth(visWidth)
    , _visHeight(visHeight)
    , _markerWidth(markerWidth)
    , _markerHeight(markerHeight)
    ,tagCodes(AprilTags::tagCodes36h11)
{
}
//-------------------------------------------------------------------------------------------------
DetectionResult TagDetector::detectTags(
    const std::vector<std::string>& filePaths, bool doCornerRefinment, bool showMarkers)
{
    using boost::property_tree::ptree;
    if (filePaths.empty()) throw std::runtime_error("Filepaths for tag detections are empty.");

    cv::Mat img = cv::imread(filePaths[0], CV_LOAD_IMAGE_GRAYSCALE);
    const int imgWidth = img.cols;
    const int imgHeight = img.rows;

    // optimized AprilTags
    // auto tagDetector=std::make_unique<AprilTags::TagDetector>(AprilTags::tagCodes36h11);
    auto tagDetector = std::unique_ptr<AprilTags::TagDetector>(
        new AprilTags::TagDetector(AprilTags::tagCodes36h11));

    const std::string processedImgsPath = _rootPath + "/" + _detectedTagsImgPath;
    if (!boost::filesystem::exists(processedImgsPath))
    {
        std::cout << "creating path: " << processedImgsPath << std::endl;
        boost::filesystem::create_directories(processedImgsPath);
    }

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
            tagObs.imageId = imageId;
            tagObs.tagId = detectedTag.id;
            tagObs.corners.resize(4);
            for (int i = 0; i < 4; ++i)
                tagObs.corners[i] << detectedTag.p[i].first, detectedTag.p[i].second;
            // do opencv corner refinement

            if (doCornerRefinment)
            {
                std::vector<cv::Point2f> corners;
                for (int i = 0; i < 4; i++)
                    corners.emplace_back(tagObs.corners[i].x(), tagObs.corners[i].y());

                cv::Size winSize = cv::Size(10, 10);
                cv::Size zeroZone = cv::Size(-1, -1);
                cv::TermCriteria criteria
                    = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.001);

                /// Calculate the refined corner locations
                cv::cornerSubPix(img, corners, winSize, zeroZone, criteria);

                for (int i = 0; i < 4; i++)
                {
                    // std::cout << "BEFOR: " << tagObs.corners[i].transpose() << std::endl;
                    tagObs.corners[i] << corners[i].x, corners[i].y;
                    // std::cout << "AFTER: " << tagObs.corners[i].transpose() << std::endl;
                }
            }

            result.tagObservations.push_back(tagObs);

            tagIds.insert(detectedTag.id);

            // Debug Visualization
            const cv::Point pos((detectedTag.p[0].first + detectedTag.p[2].first) / 2,
                (detectedTag.p[0].second + detectedTag.p[2].second) / 2);

            cv::putText(visualization, std::to_string(detectedTag.id), pos,
                cv::FONT_HERSHEY_SIMPLEX, 3.5, cv::Scalar(255, 100, 0), 10);

            for (int i = 0; i < 4; ++i)
            {
                const cv::Scalar colorMap[4] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0),
                    cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0) };
                const cv::Scalar color = colorMap[i];

                cv::circle(visualization, cv::Point2f(tagObs.corners[i].x(), tagObs.corners[i].y()),
                    img.cols * 0.002, color, 10);
            }
        }

        if (!goodObservations.empty())
        {
            std::cout << "   Detected " << goodObservations.size() << " tags: ";
            for (int i : goodObservations)
                std::cout << i << ", ";
            std::cout << std::endl;
            TagImg img;
            img.filename = boost::filesystem::path(filePath).filename().string();
            img.imageId = filteredImageId;
            result.images.push_back(img);
            filteredImageId++;
        }

        cv::imwrite(processedImgsPath + "/" + p.filename().string(), visualization);
        if (showMarkers)
        {
            cv::resize(visualization, visualization, cv::Size(_visWidth, _visHeight));
            cv::namedWindow(p.filename().string());
            cv::imshow(p.filename().string(), visualization);
            cv::waitKey(5000);
            cv::destroyWindow(p.filename().string());
        }
        imageId++;
    }

    for (int i : tagIds)
    {
        Tag tag;
        tag.tagId = i;
        tag.tagType = "apriltag_36h11"; // hardcoded for now
        tag.width = _markerWidth;
        tag.height = _markerHeight;
        result.tags.push_back(tag);
    }

    return result;
}
//-------------------------------------------------------------------------------------------------
DetectionResult TagDetector::detectTags(
    const std::string& folder, bool doCornerRefinment, bool showMarkers)
{
    std::regex reg(
        "(.*)\\.((png)|(jpg))", std::regex_constants::ECMAScript | std::regex_constants::icase);
    const std::vector<std::string> filePaths = camSurv::readFilesFromDir(folder, reg);
    return detectTags(filePaths, doCornerRefinment, showMarkers);
}
//-------------------------------------------------------------------------------------------------
}
