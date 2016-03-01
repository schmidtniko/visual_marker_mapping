/*
 *  CameraUtilities.h
 *
 *  Created on: 06.02.16
 *      Author: Stephan Manthe
 */

#ifndef CAMERAUTILITIES_H_
#define CAMERAUTILITIES_H_

#include "cameraSurveyingWithAprilTags/Camera.h"
#include "cameraSurveyingWithAprilTags/CameraModel.h"
#include "cameraSurveyingWithAprilTags/propertyTreeUtilities.h"
#include <boost/property_tree/ptree.hpp>


namespace camSurv
{
boost::property_tree::ptree cameraToPropertyTree(const Camera& camera);

Camera propertyTreeToCamera(const boost::property_tree::ptree& ptree);

boost::property_tree::ptree cameraModelToPropertyTree(const CameraModel& cameraModel);

CameraModel propertyTreeToCameraModel(const boost::property_tree::ptree& cameraModel);

CameraModel readCameraModel(const std::string& file);
}

#endif
