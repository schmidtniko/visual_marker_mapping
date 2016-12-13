#include "visual_marker_mapping/TagDetector.h"
#include "visual_marker_mapping/FileUtilities.h"
#include <boost/filesystem.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <regex>
#include <set>

namespace visual_marker_mapping
{
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
            if (tagDetection.imageId != image.imageId)
                continue;

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
