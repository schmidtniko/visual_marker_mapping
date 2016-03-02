#ifndef RECONSTRUCTIONIO_H_
#define RECONSTRUCTIONIO_H_

#include "visual_marker_mapping/TagReconstructor.h"
#include "visual_marker_mapping/CameraModel.h"
#include "visual_marker_mapping/Camera.h"
#include <string>
#include <map>

namespace camSurv
{

void exportReconstructions(const std::string& outputPath,
    const std::map<int, ReconstructedTag>& reconstructedTags,
    const std::map<int, Camera>& reconstructedCameras, const CameraModel& camModel);


void parseReconstructions(const std::string& inputPath,
    std::map<int, ReconstructedTag>& reconstructedTags, std::map<int, Camera>& reconstructedCameras,
    CameraModel& camModel);
}

#endif
