add_executable(vio-mono vio-mono.cpp)
target_link_libraries(vio-mono
        kms_common
        kms_imu
        kms_core
        ${THIRD_PARTY_LIBS})