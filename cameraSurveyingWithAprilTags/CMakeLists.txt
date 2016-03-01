cmake_minimum_required (VERSION 2.6)
project(camSurveying)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS})

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(camSurv
            src/TagReconstructor.cpp
            src/TagDetector.cpp
            src/ReconstructionIO.cpp
            src/DetectionIO.cpp
            src/CameraModel.cpp
            src/cameraUtilities.cpp
            src/eigenCVConversions.cpp
            src/fileUtilities.cpp
            src/propertyTreeUtilities.cpp)

target_link_libraries(camSurv
                      ${CERES_LIBRARIES}
                      ${Boost_LIBRARIES})


add_executable( ${PROJECT_NAME}
                ${SOURCES})

target_link_libraries(  ${PROJECT_NAME}   
                        ${OpenCV_LIBS}
                        ${CERES_LIBRARIES}
                        ${Boost_LIBRARIES}
                        apriltags)
