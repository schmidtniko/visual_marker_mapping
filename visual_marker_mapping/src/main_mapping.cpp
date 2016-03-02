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
        "project_path", po::value<std::string>()->required(), "Path to project to be processed")(
        "camera_parameter_file", po::value<std::string>()->default_value(""),
        "Path to the json file with the camera parameters.")("reconst_json_filename",
        po::value<std::string>()->default_value("reconstruction.json"),
        "Filename of the file in which the json with reconstructed Tags and Cameras is stored. It "
        "is saved in the root folder.")("json_filename",
        po::value<std::string>()->default_value("marker.json"),
        "Filename of the file in which ths json is stored. It is saved in the root folder.")(
        "start_tag_id", po::value<int>()->default_value(1)->notifier([](int param)
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
    try
    {
        boost::program_options::variables_map vm = loadParameters(argc, argv);

        const boost::filesystem::path project_path = vm["project_path"].as<std::string>();
        const std::string detection_result_filename
            = (project_path / "marker_detections.json").string();

        const std::string cam_intrinsics_file = (project_path / "camera_intrinsics.json").string();
        const int startId = vm["start_tag_id"].as<int>();
        const std::string jsonRecFilepath
            = (project_path / vm["reconst_json_filename"].as<std::string>()).string();
        const int maxThreads
            = std::thread::hardware_concurrency() ? std::thread::hardware_concurrency() : 4;

        const camSurv::CameraModel camModel = camSurv::readCameraModel(cam_intrinsics_file);

        camSurv::TagReconstructor reconstructor;
        reconstructor.readTags(detection_result_filename);
        reconstructor.setCameraModel(camModel);
        reconstructor.setOriginTagId(startId);
        reconstructor.startReconstruction(maxThreads);
    }
    catch (const std::exception& ex)
    {
        std::cout << "An exception occurred: " << ex.what() << std::endl;
    }

    return 0;
}
