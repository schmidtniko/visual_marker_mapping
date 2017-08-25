cmake_minimum_required(VERSION 3.0)
project(visual_marker_mapping_project)

SET(BUILD_UMICH ON CACHE BOOL
    "Build library by the University of Michigan")

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

# MIT AprilTags
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/fast_mit_apriltags)

# umich AprilTags
if(BUILD_UMICH)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/umich_apriltags)
endif(BUILD_UMICH)

# Ceres 
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# library dir for library targets
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib)

# bin dir for executable targets
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(WIN32)
add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DBOOST_ALL_DYN_LINK)
set(Boost_USE_STATIC_LIBS OFF )
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)
if(BUILD_UMICH)
add_definitions(-DBUILD_UMICH)
endif(BUILD_UMICH)

add_subdirectory(visual_marker_mapping)
add_subdirectory(fast_mit_apriltags EXCLUDE_FROM_ALL)

if(BUILD_UMICH)
add_subdirectory(umich_apriltags)
endif(BUILD_UMICH)
