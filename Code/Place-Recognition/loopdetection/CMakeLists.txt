cmake_minimum_required(VERSION 3.13)  
project(loop_detection)               
set(CMAKE_CXX_STANDARD 14)

# Eigen3
include_directories("/usr/include/eigen3")
find_package(Eigen3 3.1.0 REQUIRED)

find_package(Boost REQUIRED)
find_package(OpenCV 4 REQUIRED HINTS)

find_package(gflags REQUIRED)
find_package (glog 0.4.0 REQUIRED)

# include headers, ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(Torch_DIR /home/lacie/Github/libtorch/share/cmake/Torch)
find_package(Torch PATHS ${Torch_DIR} NO_DEFAULT REQUIRED)
if (Torch_FOUND)
    message(STATUS "Torch library found!")
    message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}" \n)

else ()
    message(FATAL_ERROR "Could not locate Torch" \n)
endif()
include_directories(${TORCH_INCLUDE_DIRS})

file(GLOB HEADER_FILES *.h)
file(GLOB SOURCE_FILES *.cpp)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(loop_detection ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(loop_detection
            ${OpenCV_LIBS}
            ${Boost_LIBRARIES}
            ${TORCH_LIBRARIES}
            gflags
            glog::glog)
