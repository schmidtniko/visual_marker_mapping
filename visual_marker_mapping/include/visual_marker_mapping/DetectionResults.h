#ifndef VISUAL_MARKER_MAPPING_DETECTIONRESULTS_H_
#define VISUAL_MARKER_MAPPING_DETECTIONRESULTS_H_

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
}
#endif
