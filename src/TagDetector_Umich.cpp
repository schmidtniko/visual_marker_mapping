#include "visual_marker_mapping/TagDetector_Umich.h"
#include "visual_marker_mapping/DetectionResults.h"
#include "visual_marker_mapping/FileUtilities.h"
#include <apriltag.h>
#include <boost/filesystem.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <regex>
#include <set>
#include <tag16h5.h>
#include <tag25h7.h>
#include <tag25h9.h>
#include <tag36artoolkit.h>
#include <tag36h10.h>
#include <tag36h11.h>

namespace visual_marker_mapping
{
namespace umich
{
    //-------------------------------------------------------------------------------------------------
    DetectionResult detectTags(const std::vector<std::string>& filePaths, double markerWidth,
        double markerHeight, const std::string& tagType, bool doCornerRefinement)
    {
        if (filePaths.empty())
            throw std::runtime_error("Filepaths for tag detections are empty.");

        cv::Mat img = cv::imread(filePaths[0], CV_LOAD_IMAGE_GRAYSCALE);
        const int imgWidth = img.cols;
        const int imgHeight = img.rows;

        apriltag_detector_t* td = apriltag_detector_create();
        apriltag_family_t* tf = NULL;

        if (tagType == "apriltag_16h5")
            tf = tag16h5_create();
        else if (tagType == "apriltag_25h7")
            tf = tag25h7_create();
        else if (tagType == "apriltag_25h9")
            tf = tag25h9_create();
        else if (tagType == "apriltag_36h11")
            tf = tag36h11_create();
        else if (tagType == "apriltag_36h10")
            tf = tag36h10_create();
        else if (tagType == "tag36artoolkit")
            tf = tag36artoolkit_create();
        else
        {
            apriltag_detector_destroy(td);
            throw std::runtime_error("Unsupported marker type " + tagType + "!");
        }

        tf->black_border = 1;
        apriltag_detector_add_family(td, tf);

        td->quad_decimate = 1.0;
        td->quad_sigma = 0.8;
        td->nthreads = 4;
        td->debug = 0;
        td->refine_edges = 1;
        td->refine_decode = 1;
        td->refine_pose = 0; // extremly slow and compute heavy

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
                std::cerr << "Image " << p.filename() << " has not the correct size of "
                          << imgHeight << " x " << imgWidth << std::endl;
                continue;
            }

            cv::Mat visualization = img.clone();
            cv::cvtColor(visualization, visualization, CV_GRAY2BGR);

            // Make an image_u8_t header for the Mat data
            image_u8_t im{ img.cols, img.rows, img.cols, img.data };

            zarray_t* detectedTags = apriltag_detector_detect(td, &im);

            std::set<int> goodObservations;

            for (int i = 0; i < zarray_size(detectedTags); i++)
            {
                apriltag_detection_t* detectedTag;
                zarray_get(detectedTags, i, &detectedTag);

                if (detectedTag->goodness < 0.0)
                    continue;

                goodObservations.insert(detectedTag->id);

                TagObservation tagObs;
                tagObs.imageId = filteredImageId;
                tagObs.tagId = detectedTag->id;
                tagObs.corners.resize(4);
                for (size_t i = 0; i < 4; ++i)
                    tagObs.corners[i] << detectedTag->p[i][0], detectedTag->p[i][1];

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

                tagIds.insert(detectedTag->id);
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

            zarray_destroy(detectedTags);
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

        if (tagType == "apriltag_16h5")
            tag16h5_destroy(tf);
        else if (tagType == "apriltag_25h7")
            tag25h7_destroy(tf);
        else if (tagType == "apriltag_25h9")
            tag25h9_destroy(tf);
        else if (tagType == "apriltag_36h11")
            tag36h11_destroy(tf);
        else if (tagType == "apriltag_36h10")
            tag36h10_destroy(tf);
        else if (tagType == "tag36artoolkit")
            tag36artoolkit_destroy(tf);

        apriltag_detector_destroy(td);

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
}
}
