#ifndef VISUAL_MARKER_MAPPING_FILEUTILITIES_H_
#define VISUAL_MARKER_MAPPING_FILEUTILITIES_H_

#include <vector>
#include <string>
#include <regex>

namespace camSurv
{
/**
 * Puts all files in a directory that match a regular expression into a vector.
 */
std::vector<std::string> readFilesFromDir(
    const std::string& dirPath, const std::regex& extensionFilter);
}

#endif /* FILEUTILITIES_H_ */
