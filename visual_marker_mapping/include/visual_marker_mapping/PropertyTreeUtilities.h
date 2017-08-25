#ifndef VISUAL_MARKER_MAPPING_PROPERTYTREEUTILITIES_H_
#define VISUAL_MARKER_MAPPING_PROPERTYTREEUTILITIES_H_

#include <Eigen/Core>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

#include <iostream>

namespace visual_marker_mapping
{
namespace pt = boost::property_tree;

/**
 * Reads an array of values from an property tree and puts it into an std::vector
 * @param pt A property tree whose content is an array.
 * @return A std::vector of type T with the content of the vector
 */
template <typename T> std::vector<T> as_vector(pt::ptree const& pt)
{
    std::vector<T> vec;
    for (auto& item : pt)
        vec.push_back(item.second.get_value<T>());
    return vec;
}

/**
 * Converts a Eigen matrix into an property tree.
 * @param matrix A arbitrary Eigen matrix.
 * @return The property tree
 */
template <typename Derived>
pt::ptree matrix2PropertyTreeEigen(const Eigen::MatrixBase<Derived>& matrix)
{
    constexpr bool isStaticColVector
        = (Derived::ColsAtCompileTime == 1) && (Derived::RowsAtCompileTime != Eigen::Dynamic);
    constexpr bool isStaticRowVector
        = (Derived::RowsAtCompileTime == 1) && (Derived::ColsAtCompileTime != Eigen::Dynamic);
    if (isStaticColVector || isStaticRowVector)
    {
        pt::ptree coefficents;
        for (int r = 0; r < matrix.rows(); r++)
        {
            pt::ptree coefficent;
            coefficent.put("", matrix(r));
            coefficents.push_back(std::make_pair("", coefficent));
        }
        return coefficents;
    }

    pt::ptree tree;

    tree.put("rows", matrix.rows());
    tree.put("cols", matrix.cols());

    pt::ptree coefficents;
    for (int i = 0; i < matrix.rows(); ++i)
        for (int j = 0; j < matrix.cols(); ++j)
        {
            pt::ptree coefficent;
            coefficent.put("", matrix(i, j));
            coefficents.push_back(std::make_pair("", coefficent));
        }

    tree.add_child("coefficents", coefficents);
    return tree;
}

/**
 * Converts a OpenCV matrix into an property tree.
 * @param matrix A arbitrary Eigen matrix.
 * @return The property tree
 */
template <typename T> pt::ptree matrix2PropertyTreeCv(const cv::Mat& matrix)
{
    pt::ptree tree;

    tree.put("rows", matrix.rows);
    tree.put("cols", matrix.cols);

    pt::ptree coefficents;
    for (int i = 0; i < matrix.rows; ++i)
        for (int j = 0; j < matrix.cols; ++j)
        {
            pt::ptree coefficent;
            coefficent.put("", matrix.at<T>(i, j));
            coefficents.push_back(std::make_pair("", coefficent));
        }

    tree.add_child("coefficents", coefficents);
    return tree;
}

/**
 * Converts a Matrix datastructure in a property tree into
 * a Eigen::MatrixXd.
 * @param tree The property tree
 * @return The matrix which was read
 */
template <typename EigenMatrix> EigenMatrix propertyTree2EigenMatrix(const pt::ptree& tree)
{
    constexpr bool isStaticColVector = (EigenMatrix::ColsAtCompileTime == 1)
        && (EigenMatrix::RowsAtCompileTime != Eigen::Dynamic);
    constexpr bool isStaticRowVector = (EigenMatrix::RowsAtCompileTime == 1)
        && (EigenMatrix::ColsAtCompileTime != Eigen::Dynamic);
    if (isStaticColVector || isStaticRowVector)
    {
        constexpr int numElements = EigenMatrix::ColsAtCompileTime * EigenMatrix::RowsAtCompileTime;
        EigenMatrix ret;

        int i = 0;
        for (const auto& item : tree)
        {
            if (i == numElements)
                throw std::runtime_error(
                    "Too many parameters in vector. Expected " + std::to_string(numElements) + "!");
            ret(i++) = item.second.get_value<double>();
        }
        if (i < numElements)
            throw std::runtime_error("Not enough many parameters in vector. Expected "
                + std::to_string(numElements) + ", got " + std::to_string(i) + "!");

        return ret;
    }


    int rows = tree.get<int>("rows");
    int cols = tree.get<int>("cols");

    std::vector<double> coefficents = as_vector<double>(tree.get_child("coefficents"));

    Eigen::MatrixXd matrix(rows, cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; j++)
        {
            matrix(i, j) = coefficents.at(i * cols + j);
        }

    return matrix;
}

/**
 * Converts a Matrix datastructure in a property tree into
 * a cv::Mat of type double.
 * @param tree The property tree
 * @return The matrix which was read
 */
cv::Mat propertyTree2CvMatrix(const pt::ptree& tree);
}

#endif
