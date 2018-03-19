#ifndef VISUAL_MARKER_MAPPING_RECONSTRUCTIONIO_H_
#define VISUAL_MARKER_MAPPING_RECONSTRUCTIONIO_H_

#include "visual_marker_mapping/Camera.h"
#include "visual_marker_mapping/CameraModel.h"
#include "visual_marker_mapping/TagReconstructor.h"
#include <map>
#include <string>

namespace visual_marker_mapping
{

void exportReconstructions(const std::string& outputPath,
    const std::map<int, ReconstructedTag>& reconstructedTags,
    const std::map<int, Camera>& reconstructedCameras, const CameraModel& camModel);


void parseReconstructions(const std::string& inputPath,
    std::map<int, ReconstructedTag>& reconstructedTags, std::map<int, Camera>& reconstructedCameras,
    CameraModel& camModel);
}

#endif
