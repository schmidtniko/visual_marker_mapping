#include "visual_marker_mapping/propertyTreeUtilities.h"

namespace visual_marker_mapping
{
namespace pt = boost::property_tree;
//-----------------------------------------------------------------------------
cv::Mat propertyTree2CvMatrix(const pt::ptree& tree)
{
    int rows = tree.get<int>("rows");
    int cols = tree.get<int>("cols");

    std::vector<double> coefficents = as_vector<double>(tree.get_child("coefficents"));

    cv::Mat matrix(rows, cols, CV_64FC1);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; j++)
        {
            matrix.at<double>(i, j) = coefficents.at(i * cols + j);
        }

    return matrix;
}
//-----------------------------------------------------------------------------
}
