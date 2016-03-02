#ifndef TAGDETECTOR_H_
#define TAGDETECTOR_H_

#include "AprilTags/TagDetector.h"
#include <Eigen/StdVector>
#include <string>
#include <vector>

namespace camSurv
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
    std::string filename;
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

class TagDetector
{
public:
    TagDetector(
        const std::string& detectedTagsImgPath, int visHeight, int visWidth, double markerWidth,
        double markerHeight);

    DetectionResult detectTags(const std::string& folder, bool doCornerRefinment, bool showMarkers);
    DetectionResult detectTags(
        const std::vector<std::string>& filePaths, bool doCornerRefinment, bool showMarkers);

private:
    std::string _detectedTagsImgPath;
    int _visWidth, _visHeight;
    double _markerWidth, _markerHeight;

    AprilTags::TagCodes tagCodes;
};
}

#endif
