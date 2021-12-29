# KMS_IMU
file(GLOB IMU_SOURCES imu/*.cpp)
file(GLOB IMU_HEADERS ../include/kms_slam/imu/*.h)
add_library(kms_imu SHARED ${IMU_HEADERS} ${IMU_SOURCES})
target_link_libraries(kms_imu ${THIRD_PARTY_LIBS})

# KMS_COMMON
file(GLOB COMMON_SOURCES common/*.cpp)
file(GLOB COMMON_HEADERS ../include/kms_slam/common/*.h)
add_library(kms_common SHARED ${COMMON_HEADERS} ${COMMON_SOURCES})
target_link_libraries(kms_common kms_imu ${THIRD_PARTY_LIBS})

# KMS_GTSAM
file(GLOB GTSAM_SOURCES gtsam/*.cpp)
file(GLOB GTSAM_HEADERS ../include/kms_slam/gtsam/*.h)
add_library(kms_gtsam SHARED ${GSTAM_HEADERS} ${GTSAM_SOURCES})
target_link_libraries(kms_gtsam ${THIRD_PARTY_LIBS})

# KMS_CORE
file(GLOB CORE_SOURCES   
                feature/*.cpp
                geometry/*.cpp
                optimizer/*.cpp
                system/*.cpp
                visualizer/*.cpp)
file(GLOB CORE_HEADERS
                ../include/kms_slam/feature/*.h
                ../include/kms_slam/geometry/*.h
                ../include/kms_slam/optimizer/*.h
                ../include/kms_slam/system/*.h
                ../include/kms_slam/visualizer/*.h)
add_library(kms_core SHARED ${CORE_HEADERS} ${CORE_SOURCES})
target_link_libraries(kms_core kms_imu kms_common ${THIRD_PARTY_LIBS})     
