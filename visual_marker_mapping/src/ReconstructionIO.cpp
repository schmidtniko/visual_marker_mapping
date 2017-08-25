#include "visual_marker_mapping/ReconstructionIO.h"
#include "visual_marker_mapping/CameraUtilities.h"
#include "visual_marker_mapping/PropertyTreeUtilities.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>


namespace visual_marker_mapping
{
void getRecTagTree(
    const std::map<int, ReconstructedTag>& reconstructedTags, boost::property_tree::ptree& retTree)
{
    retTree.clear();

    for (const auto& recTagPt : reconstructedTags)
    {
        const auto& actTag = recTagPt.second;

        boost::property_tree::ptree tagTree;
        tagTree.put("id", actTag.id);
        tagTree.put("type", actTag.tagType);
        tagTree.put("width", actTag.tagWidth);
        tagTree.put("height", actTag.tagHeight);

        boost::property_tree::ptree rotationTree = matrix2PropertyTreeEigen(actTag.q);
        tagTree.add_child("rotation", rotationTree);

        boost::property_tree::ptree translationTree = matrix2PropertyTreeEigen(actTag.t);
        tagTree.add_child("translation", translationTree);
        retTree.push_back(std::make_pair("", tagTree));
    }
}
//-----------------------------------------------------------------------------
void getRecCamTree(
    const std::map<int, Camera>& reconstructedCameras, boost::property_tree::ptree& retTree)
{
    retTree.clear();
    for (const auto& recCam : reconstructedCameras)
    {
        boost::property_tree::ptree camTree;
        const Camera& actCam = recCam.second;

        int id = actCam.cameraId;
        camTree.put("id", id);

        boost::property_tree::ptree qTree = matrix2PropertyTreeEigen(actCam.q);
        camTree.add_child("rotation", qTree);

        boost::property_tree::ptree tTree = matrix2PropertyTreeEigen(actCam.t);
        camTree.add_child("translation", tTree);
        retTree.push_back(std::make_pair("", camTree));
    }
}
//-----------------------------------------------------------------------------
void exportReconstructions(const std::string& outputPath,
    const std::map<int, ReconstructedTag>& reconstructedTags,
    const std::map<int, Camera>& reconstructedCameras, const CameraModel& camModel)
{
    namespace pt = boost::property_tree;
    pt::ptree mainTree;
    pt::ptree tagTree;
    getRecTagTree(reconstructedTags, tagTree);
    mainTree.add_child("reconstructed_tags", tagTree);

#if 1
    {
        pt::ptree pointTree;
        for (const auto& reconstTag : reconstructedTags)
        {
            const int tagId = reconstTag.second.id;
            const auto corners = reconstTag.second.computeMarkerCorners3D();

            for (size_t i = 0; i < 4; i++)
            {
                boost::property_tree::ptree curTree;
                curTree.put("marker_id", tagId);
                curTree.put("corner_index", i);
                const boost::property_tree::ptree tTree = matrix2PropertyTreeEigen(corners[i]);
                curTree.add_child("coords", tTree);
                pointTree.push_back(std::make_pair("", curTree));
            }
        }
        mainTree.add_child("reconstructed_marker_corners", pointTree);
    }
#endif

    pt::ptree camTree;
    getRecCamTree(reconstructedCameras, camTree);
    mainTree.add_child("reconstructed_cameras", camTree);

    pt::ptree modelTree = cameraModelToPropertyTree(camModel);
    mainTree.add_child("camera_model", modelTree);

    pt::json_parser::write_json(outputPath, mainTree);
}
//-----------------------------------------------------------------------------
std::map<int, Camera> importReconstructedCameras(const boost::property_tree::ptree& cameraArray)
{
    std::map<int, Camera> importedCameras;
    for (const auto& camPair : cameraArray)
    {
        const Camera cam = propertyTreeToCamera(camPair.second);
        importedCameras.emplace(cam.cameraId, std::move(cam));
    }

    return importedCameras;
}
//-----------------------------------------------------------------------------
std::map<int, ReconstructedTag> importReconstructedTags(const boost::property_tree::ptree& tagArray)
{
    std::map<int, ReconstructedTag> reconstructedTags;
    namespace pt = boost::property_tree;
    for (const auto& tagPt : tagArray)
    {
        const auto& tag = tagPt.second;
        int id = tag.get<int>("id");
        reconstructedTags[id].id = id;
        reconstructedTags[id].q
            = propertyTree2EigenMatrix<Eigen::Vector4d>(tag.get_child("rotation"));
        reconstructedTags[id].t
            = propertyTree2EigenMatrix<Eigen::Vector3d>(tag.get_child("translation"));
        reconstructedTags[id].tagWidth = tag.get<double>("width");
        reconstructedTags[id].tagHeight = tag.get<double>("height");
        reconstructedTags[id].tagType = tag.get<std::string>("type");
    }
    // std::cout << "size = " << reconstructedTags.size() << std::endl;
    return reconstructedTags;
}
//-----------------------------------------------------------------------------
void parseReconstructions(const std::string& inputPath,
    std::map<int, ReconstructedTag>& reconstructedTags, std::map<int, Camera>& reconstructedCameras,
    visual_marker_mapping::CameraModel& camModel)
{
    reconstructedCameras.clear();
    reconstructedTags.clear();

    boost::property_tree::ptree rootNode;
    boost::property_tree::json_parser::read_json(inputPath, rootNode);

    reconstructedTags = importReconstructedTags(rootNode.get_child("reconstructed_tags"));
    reconstructedCameras = importReconstructedCameras(rootNode.get_child("reconstructed_cameras"));
    camModel = propertyTreeToCameraModel(rootNode.get_child("camera_model"));
}
//-----------------------------------------------------------------------------
}
