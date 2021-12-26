cmake_minimum_required(VERSION 3.2)
project(kms_slam)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 ")

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

# GLEW
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
link_libraries(${GLEW_LIBRARIES})

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


set (THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_core.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_types_sba.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_csparse_extension.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_solver_dense.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_solver_eigen.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_types_sim3.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_types_slam3d.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o_lib/libg2o_types_slam2d.so
        cholmod
        ${CHOLMOD_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        GLEW::GLEW)

add_subdirectory(src)
add_subdirectory(Examples/VIO)
