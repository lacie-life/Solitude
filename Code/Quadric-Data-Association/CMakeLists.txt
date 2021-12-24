project(gtsam_quadrics)
cmake_minimum_required(VERSION 3.0)
set(GTSAM_QUADRICS_VERSION 1.0.0)


# we want to use some C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

################################################################################
# add gtsam_quadrics

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Boost REQUIRED)

# include headers, ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

# set source files
set(SOURCE_FILES
  ./gtsam_quadrics/base/Utilities.cpp
  ./gtsam_quadrics/geometry/ConstrainedDualQuadric.cpp
  ./gtsam_quadrics/geometry/AlignedBox2.cpp
  ./gtsam_quadrics/geometry/AlignedBox3.cpp
  ./gtsam_quadrics/geometry/BoundingBoxFactor.cpp
  ./gtsam_quadrics/geometry/QuadricAngleFactor.cpp
  ./gtsam_quadrics/geometry/QuadricCamera.cpp
  ./gtsam_quadrics/geometry/DualConic.cpp
  )

###################################################################################
## Build static library from common sources

set(CONVENIENCE_LIB_NAME "gtsam_quadrics")
add_library(${CONVENIENCE_LIB_NAME} SHARED ${SOURCE_FILES})
# set_target_properties(${CONVENIENCE_LIB_NAME} PROPERTIES PREFIX "")
target_link_libraries(${CONVENIENCE_LIB_NAME}
        gtsam
        ${Boost_LIBRARIES}
        -lboost_system)

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

