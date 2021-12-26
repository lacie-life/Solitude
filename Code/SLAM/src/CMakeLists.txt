# KMS_IMU
file(GLOB IMU_SOURCES imu/*.cpp)
add_library(kms_imu SHARED ${IMU_SOURCES})
target_link_libraries(kms_imu ${THIRD_PARTY_LIBS})

# KMS_COMMON
file(GLOB COMMON_SOURCES common/*.cpp)
add_library(kms_common SHARED ${COMMON_SOURCES})
target_link_libraries(kms_common kms_imu ${THIRD_PARTY_LIBS})

# KMS_CORE
file(GLOB CORE_SOURCES   
                feature/*.cpp
                geometry/*.cpp
                optimizer/*.cpp
                system/*.cpp
                visualizer/*.cpp)
add_library(kms_core SHARED ${CORE_SOURCES})
target_link_libraries(kms_core kms_imu kms_common ${THIRD_PARTY_LIBS})     
