#include "visual_marker_mapping/cameraUtilities.h"
#include "visual_marker_mapping/propertyTreeUtilities.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace visual_marker_mapping
{
//-----------------------------------------------------------------------------
boost::property_tree::ptree cameraToPropertyTree(const Camera& camera)
{
    namespace pt = boost::property_tree;
    pt::ptree cameraPt;

    cameraPt.add_child("t", matrix2PropertyTreeEigen(camera.t));
    cameraPt.add_child("q", matrix2PropertyTreeEigen(camera.q));

    return cameraPt;
}
//-----------------------------------------------------------------------------
Camera propertyTreeToCamera(const boost::property_tree::ptree& ptree)
{
    Camera cam;
    cam.t = propertyTree2EigenMatrix<Eigen::Vector3d>(ptree.get_child("t"));
    cam.q = propertyTree2EigenMatrix<Eigen::Vector4d>(ptree.get_child("q"));

    return cam;
}
//-----------------------------------------------------------------------------
boost::property_tree::ptree cameraModelToPropertyTree(const CameraModel& cameraModel)
{
    namespace pt = boost::property_tree;
    pt::ptree cameraModelPt;
    cameraModelPt.put("fx", cameraModel.fx);
    cameraModelPt.put("fy", cameraModel.fy);
    cameraModelPt.put("cx", cameraModel.cx);
    cameraModelPt.put("cy", cameraModel.cy);
    cameraModelPt.add_child(
        "distortion_coefficients", matrix2PropertyTreeEigen(cameraModel.distortionCoefficients));
    cameraModelPt.put("vertical_resolution", cameraModel.verticalResolution);
    cameraModelPt.put("horizontal_resolution", cameraModel.horizontalResolution);
    return cameraModelPt;
}
//-----------------------------------------------------------------------------
CameraModel propertyTreeToCameraModel(const boost::property_tree::ptree& ptree)
{
    CameraModel model;
    model.fx = ptree.get<double>("fx");
    model.fy = ptree.get<double>("fy");
    model.cx = ptree.get<double>("cx");
    model.cy = ptree.get<double>("cy");

    model.distortionCoefficients = propertyTree2EigenMatrix<Eigen::Matrix<double, 5, 1> >(
        ptree.get_child("distortion_coefficients"));
    model.horizontalResolution = ptree.get<int>("horizontal_resolution");
    model.verticalResolution = ptree.get<int>("vertical_resolution");
    return model;
}
//-----------------------------------------------------------------------------
CameraModel readCameraModel(const std::string& file)
{
    namespace pt = boost::property_tree;
    pt::ptree cameraModelPt;
    pt::read_json(file, cameraModelPt);

    return propertyTreeToCameraModel(cameraModelPt);
}
//-----------------------------------------------------------------------------
}
