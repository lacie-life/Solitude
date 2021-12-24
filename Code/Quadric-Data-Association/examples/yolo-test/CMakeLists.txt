# find gtsam for eigen
find_package(GTSAM REQUIRED HINTS /home/jun/Github/Master-Thesis/Code/Quadric-Data-Association/gtsam)

# include project headers
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

add_executable(yolo-test main.cpp)

# link with gtsam (or recieve undefined refrences)
target_link_libraries(yolo-test
        gtsam
        ${CONVENIENCE_LIB_NAME}
        ${Boost_LIBRARIES})
