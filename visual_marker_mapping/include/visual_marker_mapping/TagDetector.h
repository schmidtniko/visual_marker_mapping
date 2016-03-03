#ifndef VISUAL_MARKER_MAPPING_TAGDETECTOR_H_
#define VISUAL_MARKER_MAPPING_TAGDETECTOR_H_

#include <Eigen/StdVector>
#include <string>
#include <vector>

namespace visual_marker_mapping
{
struct TagObservation
{
    int imageId = -1;
    int tagId = -1;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        corners; // observed Tag corners
};

struct TagImg
{
    int imageId = -1;
    std::string filePath;
};

struct Tag
{
    int tagId;
    std::string tagType;
    double width;
    double height;
};

struct DetectionResult
{
    std::vector<TagImg> images;
    std::vector<Tag> tags;
    std::vector<TagObservation> tagObservations;
};

DetectionResult detectTags(const std::string& folder, double markerWidth, double markerHeight,
    const std::string& tagType = "apriltag_36h11", bool doCornerRefinement = false);
DetectionResult detectTags(const std::vector<std::string>& filePaths, double markerWidth,
    double markerHeight, const std::string& tagType = "apriltag_36h11",
    bool doCornerRefinement = false);

void visualizeTagResult(const DetectionResult& detectionResult, const std::string& exportFolder);
}

#endif
