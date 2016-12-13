#ifndef VISUAL_MARKER_MAPPING_TAGDETECTOR_H_
#define VISUAL_MARKER_MAPPING_TAGDETECTOR_H_

#include "DetectionResults.h"
#include <Eigen/StdVector>
#include <string>
#include <vector>

namespace visual_marker_mapping
{
void visualizeDetectionResults(
    const DetectionResult& detectionResult, const std::string& exportFolder);
}

#endif
