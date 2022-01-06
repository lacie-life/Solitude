project(gtsam_quadrics)
cmake_minimum_required(VERSION 3.0)
set(GTSAM_QUADRICS_VERSION 1.0.0)


# we want to use some C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################################################################################
# add gtsam_quadrics

# Eigen3
include_directories("/usr/include/eigen3")
find_package(Eigen3 3.1.0 REQUIRED)

find_package(Boost REQUIRED)
find_package(OpenCV 4 REQUIRED)

# include headers, ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# find GTSAM and GTSAM CMake tools
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools REQUIRED)

# include gtsam cmake tools
include(GtsamBuildTypes) 			# Load build type flags and default to Debug mode
include(GtsamTesting)    			# Easy functions for creating unit tests and scripts
include(GtsamMakeConfigFile) 	# Defines a function for generating config file so package can be found with find_package function

set(Torch_DIR /home/lacie/Github/libtorch/share/cmake/Torch)
find_package(Torch PATHS ${Torch_DIR} NO_DEFAULT REQUIRED)
if (Torch_FOUND)
    message(STATUS "Torch library found!")
    message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}" \n)

else ()
    message(FATAL_ERROR "Could not locate Torch" \n)
endif()
include_directories(${TORCH_INCLUDE_DIRS})

# set source files
set(SOURCE_FILES
    gtsam_quadrics/base/Utilities.cpp
    gtsam_quadrics/geometry/ConstrainedDualQuadric.cpp
    gtsam_quadrics/geometry/AlignedBox2.cpp
    gtsam_quadrics/geometry/AlignedBox3.cpp
    gtsam_quadrics/geometry/BoundingBoxFactor.cpp
    gtsam_quadrics/geometry/QuadricAngleFactor.cpp
    gtsam_quadrics/geometry/QuadricCamera.cpp
    gtsam_quadrics/geometry/DualConic.cpp
    gtsam_quadrics/models/yolo.cpp
  )

set(THIRD_PARTY_LIBS
        gtsam
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${TORCH_LIBRARIES}
        -lboost_system)

###################################################################################
## Build static library from common sources

set(CONVENIENCE_LIB_NAME "gtsam_quadrics")
add_library(${CONVENIENCE_LIB_NAME} SHARED ${SOURCE_FILES})
# set_target_properties(${CONVENIENCE_LIB_NAME} PROPERTIES PREFIX "")
target_link_libraries(${CONVENIENCE_LIB_NAME}
        gtsam
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${TORCH_LIBRARIES}
        -lboost_system)

set_property(TARGET ${CONVENIENCE_LIB_NAME} PROPERTY CXX_STANDARD 14)

###################################################################################
# Build example files (CMake tracks the dependecy to link with GTSAM through our project's static library)
# TODO fix broken examples!
# gtsamAddExamplesGlob("*.cpp" "" "${CONVENIENCE_LIB_NAME}")
# build examples if make examples
# https://stackoverflow.com/questions/17765618/cmake-add-subdirectories-to-custom-target-and-adding-custom-target-to-vs-solut
# TODO: pass ${CONVENIENCE_LIB_NAME} to examples/c++/CMakeLists.txt
# add_custom_target(examples)
# add_subdirectory(examples/c++)
add_subdirectory(examples/yolo-test)

