#ifndef RECONSTRUCTIONIO_H_
#define RECONSTRUCTIONIO_H_

#include "cameraSurveyingWithAprilTags/TagReconstructor.h"
#include "cameraSurveyingWithAprilTags/CameraModel.h"
#include "cameraSurveyingWithAprilTags/Camera.h"
#include <string>
#include <map>

namespace camSurv
{

void exportReconstructions(const std::string &outputPath,
                              const std::map<int, ReconstructedTag>& reconstructedTags,
                              const std::map<int, Camera>& reconstructedCameras,
                              const CameraModel& camModel);


void parseReconstructions(const std::string& inputPath,
                              std::map<int, ReconstructedTag>& reconstructedTags,
                              std::map<int, Camera>& reconstructedCameras,
                              CameraModel& camModel);
}

#endif
