cmake_minimum_required (VERSION 3.1)
project(visual_marker_mapping)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang") # Works for Clang and AppleClang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

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
                      apriltags)
					  
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
                      visual_marker_mapping_lib
                      apriltags)

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
                      visual_marker_mapping_lib
                      apriltags)
if(BUILD_UMICH)
target_link_libraries(visual_marker_detection umich_apriltags)
endif(BUILD_UMICH)