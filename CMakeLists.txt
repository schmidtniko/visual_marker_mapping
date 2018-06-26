cmake_minimum_required(VERSION 3.0)
project(visual_marker_mapping)

SET(BUILD_UMICH ON CACHE BOOL
    "Build library by the University of Michigan")

if(BUILD_UMICH)
add_definitions(-DBUILD_UMICH)
endif(BUILD_UMICH)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang") # Works for Clang and AppleClang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

# Find Boost
find_package(Boost COMPONENTS filesystem system program_options REQUIRED) 
include_directories(${Boost_INCLUDE_DIR})
if(WIN32)
add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DBOOST_ALL_DYN_LINK)
set(Boost_USE_STATIC_LIBS OFF )
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

# Ceres 
find_package(Ceres REQUIRED HINTS /usr/lib/cmake/Ceres)

# bin dir for executable targets
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


###############################################################################

add_library(visual_marker_mapping_lib
            src/TagReconstructor.cpp
            src/TagDetector.cpp
            src/ReconstructionIO.cpp
            src/DetectionIO.cpp
            src/CameraModel.cpp
            src/CameraUtilities.cpp
            src/EigenCVConversions.cpp
            src/FileUtilities.cpp
            src/PropertyTreeUtilities.cpp
            src/TagDetector_MIT.cpp
            include/visual_marker_mapping/Camera.h
            include/visual_marker_mapping/TagReconstructor.h
            include/visual_marker_mapping/DetectionIO.h
            include/visual_marker_mapping/TagDetector.h
            include/visual_marker_mapping/PropertyTreeUtilities.h
            include/visual_marker_mapping/ReconstructionIO.h
            include/visual_marker_mapping/EigenCVConversions.h
            include/visual_marker_mapping/FileUtilities.h
            include/visual_marker_mapping/CameraModel.h
            include/visual_marker_mapping/CameraUtilities.h
            include/visual_marker_mapping/TagReconstructionCostFunction.h
            include/visual_marker_mapping/DetectionResults.h
            include/visual_marker_mapping/TagDetector_MIT.h)

if(BUILD_UMICH)
target_sources(visual_marker_mapping_lib PRIVATE
			src/TagDetector_Umich.cpp
			include/visual_marker_mapping/TagDetector_Umich.h)
endif(BUILD_UMICH)
			
set_property(TARGET visual_marker_mapping_lib PROPERTY CXX_STANDARD 11)
set_property(TARGET visual_marker_mapping_lib PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(visual_marker_mapping_lib PUBLIC
                           include
                           ${Boost_INCLUDE_DIRS}
                           ${CERES_INCLUDE_DIRS})

target_link_libraries(visual_marker_mapping_lib
                      ${OpenCV_LIBS}
                      ${CERES_LIBRARIES}
                      ${Boost_LIBRARIES}
                      Eigen3::Eigen
                      ${FLEP_PACKAGE_DEPS})
					  
if(BUILD_UMICH)
target_link_libraries(visual_marker_mapping_lib umich_apriltags)
endif(BUILD_UMICH)

###############################################################################

add_executable(visual_marker_mapping
               src/main_mapping.cpp)

set_property(TARGET visual_marker_mapping PROPERTY CXX_STANDARD 11)
set_property(TARGET visual_marker_mapping PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(visual_marker_mapping_lib PUBLIC
                           include
                           ${Boost_INCLUDE_DIRS}
                           ${CERES_INCLUDE_DIRS})

target_link_libraries(visual_marker_mapping
                      ${OpenCV_LIBS}
                      ${CERES_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GLOG_LIBRARY}
                      Eigen3::Eigen
                      visual_marker_mapping_lib
                      ${FLEP_PACKAGE_DEPS})

###############################################################################

add_executable(visual_marker_detection
               src/main_detection.cpp)

set_property(TARGET visual_marker_detection PROPERTY CXX_STANDARD 11)
set_property(TARGET visual_marker_detection PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(visual_marker_mapping_lib PUBLIC
                           include
                           ${Boost_INCLUDE_DIRS}
                           ${CERES_INCLUDE_DIRS})

target_link_libraries(visual_marker_detection
                      ${OpenCV_LIBS}
                      ${CERES_LIBRARIES}
                      ${Boost_LIBRARIES}
                      Eigen3::Eigen
                      visual_marker_mapping_lib
                      ${FLEP_PACKAGE_DEPS})
if(BUILD_UMICH)
target_link_libraries(visual_marker_detection umich_apriltags)
endif(BUILD_UMICH)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_DONT_ALIGN_STATICALLY)
    target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
endif()
