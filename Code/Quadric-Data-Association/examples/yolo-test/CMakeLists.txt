
# include project headers
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find GTSAM and GTSAM CMake tools
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

add_executable(yolo-test main.cpp)

# link with gt-sam (or receive undefined references)
target_link_libraries(yolo-test
        gtsam
        ${CONVENIENCE_LIB_NAME}
        ${OpenCV_LIBS}
        ${THIRD_PARTY_LIBS})
