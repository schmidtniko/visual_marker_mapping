#ifndef VISUAL_MARKER_MAPPING_TAGDETECTOR_MIT_H_
#define VISUAL_MARKER_MAPPING_TAGDETECTOR_MIT_H_

#include "DetectionResults.h"
#include <Eigen/StdVector>
#include <string>
#include <vector>

namespace visual_marker_mapping
{
namespace mit
{
    DetectionResult detectTags(const std::string& folder, double markerWidth, double markerHeight,
        const std::string& tagType = "apriltag_36h11", bool doCornerRefinement = false);
    DetectionResult detectTags(const std::vector<std::string>& filePaths, double markerWidth,
        double markerHeight, const std::string& tagType = "apriltag_36h11",
        bool doCornerRefinement = false);
}
}

#endif
