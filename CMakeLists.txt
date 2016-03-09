cmake_minimum_required(VERSION 3.0)
project(visual_marker_mapping_project)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Boost
find_package(Boost COMPONENTS filesystem system program_options REQUIRED) 
include_directories(${Boost_INCLUDE_DIR})

# AprilTags
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/fast_mit_apriltags)

# Ceres 
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# library dir for library targets
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib)

# bin dir for executable targets
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(visual_marker_mapping)
add_subdirectory(fast_mit_apriltags)
