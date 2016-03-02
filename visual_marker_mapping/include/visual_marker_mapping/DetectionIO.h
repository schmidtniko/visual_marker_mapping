#ifndef VISUAL_MARKER_MAPPING_DETECTIONIO_H
#define VISUAL_MARKER_MAPPING_DETECTIONIO_H

#include "TagDetector.h"
#include <string>

namespace camSurv
{
DetectionResult readDetectionResult(const std::string& filename);
bool writeDetectionResult(const DetectionResult& result, const std::string& filename);
}

#endif