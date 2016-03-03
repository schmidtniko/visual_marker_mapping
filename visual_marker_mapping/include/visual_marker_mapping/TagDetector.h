#ifndef VISUAL_MARKER_MAPPING_TAGDETECTOR_H_
#define VISUAL_MARKER_MAPPING_TAGDETECTOR_H_

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

class TagDetector
{
public:
    TagDetector(
        int visHeight, int visWidth, double markerWidth,
        double markerHeight);

    DetectionResult detectTags(const std::string& folder, bool doCornerRefinement);
    DetectionResult detectTags(
        const std::vector<std::string>& filePaths, bool doCornerRefinement);

    void visualizeTagResult(const DetectionResult& detectionResult, const std::string& exportFolder) const;


private:
    int _visWidth, _visHeight;
    double _markerWidth, _markerHeight;
};
}

#endif
