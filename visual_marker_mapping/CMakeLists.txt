cmake_minimum_required (VERSION 2.6)
project(visual_marker_mapping)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

add_library(visual_marker_mapping_lib
            src/TagReconstructor.cpp
            src/TagDetector.cpp
            src/ReconstructionIO.cpp
            src/DetectionIO.cpp
            src/CameraModel.cpp
            src/cameraUtilities.cpp
            src/eigenCVConversions.cpp
            src/fileUtilities.cpp
            src/propertyTreeUtilities.cpp)

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
                      apriltags)

add_executable(visual_marker_mapping
               src/main.cpp)

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
                      visual_marker_mapping_lib
                      apriltags)

