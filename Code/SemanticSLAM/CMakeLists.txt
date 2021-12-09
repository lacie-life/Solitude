cmake_minimum_required(VERSION 3.20)
project(SematicSLAM)

set(sematicSLAM_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)

# we want to use some C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_LIBRARY_PATH /opt/intel/mkl/lib/)
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen3
include_directories("/usr/include/eigen3")
find_package (Eigen3 3.3 REQUIRED)

# set up options
option(BUILD_PYTHON_WRAP "Enable/Disable building of cython wrapper" ON)
set(PYTHON_VERSION "3" CACHE STRING "The version of python to build the cython wrapper for (or Default)")

###################################################################################
# find GTSAM and GTSAM CMake tools
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools REQUIRED)

# include gtsam cmake tools
include(GtsamBuildTypes) 			# Load build type flags and default to Debug mode
include(GtsamTesting)    			# Easy functions for creating unit tests and scripts
include(GtsamMakeConfigFile) 	# Defines a function for generating config file so package can be found with find_package function

set(Torch_DIR /home/jun/Github/libtorch/share/cmake/Torch)
find_package(Torch PATHS ${Torch_DIR} NO_DEFAULT REQUIRED)
if (Torch_FOUND)
    message(STATUS "Torch library found!")
    message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}" \n)

else ()
    message(FATAL_ERROR "Could not locate Torch" \n)
endif()
include_directories(${TORCH_INCLUDE_DIRS})

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

set(SOURCE_FILES
        sematicSLAM/sematicBase/QuadricProjectionException.cpp
        sematicSLAM/sematicBase/Utilities.cpp
        sematicSLAM/sematicBase/yolo.cpp
        sematicSLAM/core/Camera.cpp
        sematicSLAM/core/ConstrainedDualQuadric.cpp
        sematicSLAM/core/AlignedBox2.cpp
        sematicSLAM/core/AlignedBox3.cpp
        sematicSLAM/core/DualConic.cpp
        sematicSLAM/core/BoundingBoxFactor.cpp
        sematicSLAM/core/QuadricAngleFactor.cpp
        sematicSLAM/vo/bucket.cpp
        sematicSLAM/vo/evaluate_odometry.cpp
        sematicSLAM/vo/feature.cpp
        sematicSLAM/vo/frame.cpp
        sematicSLAM/vo/matrix.cpp
        sematicSLAM/vo/utils.cpp
        sematicSLAM/vo/visualOdometry.cpp
        sematicSLAM/imu/imudata.cpp
        sematicSLAM/imu/IMUPreintegrator.cpp
        sematicSLAM/imu/so3.h
        sematicSLAM/imu/se3.hpp
        sematicSLAM/imu/so3.hpp
        sematicSLAM/imu/sophus.hpp)

set(HEADER_FILES
        sematicSLAM/sematicslam.h
        sematicSLAM/sematicBase/QuadricProjectionException.h
        sematicSLAM/sematicBase/Utilities.h
        sematicSLAM/sematicBase/yolo.h
        sematicSLAM/core/Camera.h
        sematicSLAM/core/ConstrainedDualQuadric.h
        sematicSLAM/core/AlignedBox2.h
        sematicSLAM/core/AlignedBox3.h
        sematicSLAM/core/DualConic.h
        sematicSLAM/core/QuadricAngleFactor.h
        sematicSLAM/core/BoundingBoxFactor.h
        ematicSLAM/vo/bucket.h
        sematicSLAM/vo/camera.h
        sematicSLAM/vo/evaluate_odometry.h
        sematicSLAM/vo/feature.h
        sematicSLAM/vo/frame.h
        sematicSLAM/vo/matrix.h
        sematicSLAM/vo/utils.h
        sematicSLAM/vo/visualOdometry.h
        sematicSLAM/imu/imudata.h
        sematicSLAM/imu/IMUPreintegrator.h)

set(CONVENIENCE_LIB_NAME "semanticSLAM")
add_library(${CONVENIENCE_LIB_NAME} ${SOURCE_FILES})
# set_target_properties(${CONVENIENCE_LIB_NAME} PROPERTIES PREFIX "")
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam
                                              ${OpenCV_LIBS}
                                              ${TORCH_LIBRARIES})

set_property(TARGET ${CONVENIENCE_LIB_NAME} PROPERTY CXX_STANDARD 14)

add_subdirectory(examples)
