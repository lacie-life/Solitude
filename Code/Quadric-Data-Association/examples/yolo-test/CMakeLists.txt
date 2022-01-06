
# include project headers
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(yolo-test main.cpp)

# link with gtsam (or recieve undefined refrences)
target_link_libraries(yolo-test
        gtsam
        ${CONVENIENCE_LIB_NAME}
        ${OpenCV_LIBS}
        ${THIRD_PARTY_LIBS})
