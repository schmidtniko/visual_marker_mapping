#include <ceres/ceres.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include "visual_marker_mapping/TagDetector.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include "visual_marker_mapping/cameraUtilities.h"
#include "visual_marker_mapping/TagReconstructor.h"
#include "visual_marker_mapping/DetectionIO.h"

//------------------------------------------------------------------------------------------------------------
template <typename T>
void checkRange(
    T param, std::string paramName, T lowerBound = 0, T upperBound = std::numeric_limits<T>::max())
{
    if (param < lowerBound)
        throw std::runtime_error(
            "Value for " + paramName + " is smaller than " + std::to_string(lowerBound) + " .");
    if (param > upperBound)
        throw std::runtime_error("Value for parameter " + paramName + " is bigger than "
            + std::to_string(upperBound) + " .");
}
//------------------------------------------------------------------------------------------------------------
namespace po = boost::program_options;
po::variables_map loadParameters(int argc, char* argv[])
{
    po::options_description options("Allowed options");
    po::options_description fileOptions("Options for the config file");
    options.add_options()("help,?", "produces this help message")(
        "config_file", po::value<std::string>()->required(), "Path to the config ini file")(
        "do_reconstruction", po::bool_switch()->default_value(false), "")(
        "show_tags", po::bool_switch()->default_value(false), "Visualizes the detected markers.")(
        "do_corner_refinment", po::bool_switch()->default_value(false),
        "If this option is set an additional corner refinment of the marker corners is done.");

    fileOptions.add_options()("camera_parameter_file", po::value<std::string>()->default_value(""),
        "Path to the xml file with the camera parameters.")("tag_img_path",
        po::value<std::string>()->default_value("."),
        "Path to the folder with the images of the tags.")("tag_detections_path",
        po::value<std::string>()->default_value("dbg_tag_detections"),
        "Path to the folder where the processed images are stored.")("reconst_json_filename",
        po::value<std::string>()->default_value("reconstructedObjects.json"),
        "Filename of the file in which the json with reconstructed Tags and Cameras is stored. It "
        "is saved in the root folder.")("json_filename",
        po::value<std::string>()->default_value("marker.json"),
        "Filename of the file in which ths json is stored. It is saved in the root folder.");

    fileOptions.add_options()("visualization_width",
        po::value<int>()->default_value(1500)->notifier([](int param)
            {
                checkRange(param, "visualization_width", 640);
            }),
        "Width of an tag measured parallel to the tag name.")("visualization_height",
        po::value<int>()->default_value(1000)->notifier([](int param)
            {
                checkRange<int>(param, "visualization_height", 480);
            }),
        "Height of an tag measured perpendicular to the tag name.")("tag_width",
        po::value<double>()->default_value(0.116)->notifier([](int param)
            {
                checkRange<double>(param, "tag_width", 0.0);
            }),
        "Width of an tag measured parallel to the tag name.")("tag_height",
        po::value<double>()->default_value(0.117)->notifier([](int param)
            {
                checkRange<double>(param, "tag_height", 0.0);
            }),
        "Height of an tag measured perpendicular to the tag name.")("max_threads",
        po::value<int>()->default_value(1)->notifier([](int param)
            {
                checkRange<int>(param, "max_threads", 0);
            }),
        "The number of threads with which the program is working. Currently this parameter is only "
        "used for Ceres.")("start_tag_id",
        po::value<int>()->default_value(1)->notifier([](int param)
            {
                checkRange<int>(param, "start_tag_id", 0);
            }),
        "Id of the marker which will be in the origin of the model.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, options), vm);
    if (vm.count("help"))
    {
        std::cout << options << std::endl;
        std::cout << fileOptions << std::endl;
        exit(0);
    }

    po::notify(vm);

    std::string configFilePath = vm["config_file"].as<std::string>();
    std::ifstream ini_file(configFilePath);

    if (!ini_file) throw std::runtime_error("Could not open the config file: " + configFilePath);

    po::store(po::parse_config_file(ini_file, fileOptions, true), vm);
    po::notify(vm);

    return vm;
}
//------------------------------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    boost::program_options::variables_map vm = loadParameters(argc, argv);
    boost::filesystem::path p(vm["config_file"].as<std::string>());
    std::string projectPath = p.parent_path().string();
    // std::string rootPath = vm["root_path"].as<std::string>();

    if (vm["do_reconstruction"].as<bool>())
    {
        const std::string camFilePath = projectPath + vm["camera_parameter_file"].as<std::string>();
        const std::string jsonFilepath = projectPath + "/" + vm["json_filename"].as<std::string>();
        const int startId = vm["start_tag_id"].as<int>();
        const std::string jsonRecFilepath
            = projectPath + "/" + vm["reconst_json_filename"].as<std::string>();
        const int maxThreads = vm["max_threads"].as<int>();


        camSurv::CameraModel camModel = camSurv::readCameraModel(camFilePath);

        camSurv::TagReconstructor reconstructor;
        reconstructor.readTags(jsonFilepath);
        reconstructor.setCameraModel(camModel);
        reconstructor.setOriginTagId(startId);
        reconstructor.startReconstruction(maxThreads);
    }
    else
    {
        auto tp = std::chrono::system_clock::now();

        const std::string jsonFilename = vm["json_filename"].as<std::string>();
        const std::string jsonRecFilepath
            = projectPath + "/" + vm["reconst_json_filename"].as<std::string>();
        const std::string markerImgPath = projectPath + vm["tag_img_path"].as<std::string>();
        const std::string imgPath = vm["tag_img_path"].as<std::string>();
        const std::string tagDetectionsPath = vm["tag_detections_path"].as<std::string>();
        camSurv::TagDetector tagDetector(projectPath, imgPath, tagDetectionsPath,
            vm["visualization_height"].as<int>(), vm["visualization_width"].as<int>(),
            vm["tag_width"].as<double>(), vm["tag_height"].as<double>());

        const std::string imgFolder = projectPath + "/" + imgPath;
        const auto result = tagDetector.detectTags(
            imgFolder, vm["do_corner_refinment"].as<bool>(), vm["show_tags"].as<bool>());
        camSurv::writeDetectionResult(result, projectPath + "/" + jsonFilename);

        auto tp2 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = tp2 - tp;
        std::cout << "elapsed_seconds = " << elapsed_seconds.count() << std::endl;
    }
    return 0;
}
