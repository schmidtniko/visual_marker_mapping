#include "visual_marker_mapping/FileUtilities.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <regex>

namespace visual_marker_mapping
{
std::vector<std::string> readFilesFromDir(
    const std::string& dirPath, const std::regex& extensionFilter)
{
    namespace fs = boost::filesystem;
    fs::path someDir(dirPath);

    if (!fs::exists(someDir))
        throw std::runtime_error("Directory: " + dirPath + " is does not exist.");

    if (!fs::is_directory(someDir))
        throw std::runtime_error("Path: " + dirPath + " is not an directory.");

    fs::directory_iterator end_iter;
    std::vector<std::string> files;
    for (fs::directory_iterator dir_iter(someDir); dir_iter != end_iter; ++dir_iter)
    {
        if (!fs::is_regular_file(dir_iter->status()))
            continue;

        const std::string path = dir_iter->path().string();
        try
        {
            std::smatch matchResults;
            if (!std::regex_match(path, matchResults, extensionFilter))
                continue;
        }
        catch (const std::regex_error& e)
        {
            std::cerr << e.what() << std::endl;
            continue;
        }

        files.push_back(path);
    }
    return files;
}
}
