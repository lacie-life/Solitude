cmake_minimum_required(VERSION 3.2)
project(kms_slam)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)

# G2O
find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("/usr/local/include/")
LINK_directories("/usr/local/lib")

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include/kms_slam
        ${PROJECT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

file(GLOB HEADERS
        ${PROJECT_SOURCE_DIR}/include/kms_slam/common/*.h
        ${PROJECT_SOURCE_DIR}/include/kms_slam/feature/*.h
        ${PROJECT_SOURCE_DIR}/include/kms_slam/geometry/*.h
        ${PROJECT_SOURCE_DIR}/include/kms_slam/optimizer/*.h
        ${PROJECT_SOURCE_DIR}/include/kms_slam/system/*.h
        ${PROJECT_SOURCE_DIR}/include/kms_slam/visualizer/*.h)

file(GLOB SOURCES
        ${PROJECT_SOURCE_DIR}/src/common/*.cpp
        ${PROJECT_SOURCE_DIR}/src/feature/*.cpp
        ${PROJECT_SOURCE_DIR}/src/geometry/*.cpp
        ${PROJECT_SOURCE_DIR}/src/optimizer/*.cpp
        ${PROJECT_SOURCE_DIR}/src/system/*.cpp
        ${PROJECT_SOURCE_DIR}/src/visualizer/*.cpp)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

add_library(${PROJECT_NAME} SHARED
        ${HEADERS}
        ${SOURCES})

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_core.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_types_sba.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_csparse_extension.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_solver_dense.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_solver_eigen.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_types_sim3.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_types_slam3d.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o_lib/lib/libg2o_types_slam2d.so
        ${PROJECT_SOURCE_DIR}/thirdparty/fast/build/libfast.so
        cholmod
        ${CHOLMOD_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES})

# set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_RELEASE -s)

## Build VIO test

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/VIO)

add_executable(vio-mono Examples/VIO/vio-mono.cpp)
target_link_libraries(vio-mono ${PROJECT_NAME})

## Build examples

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})


#add_executable(stereo_euroc
#Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

#add_executable(mono_euroc
#Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})

## Build tools
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(bin_vocabulary
#tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME})
