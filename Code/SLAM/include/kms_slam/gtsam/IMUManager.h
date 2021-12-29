//
// Created by lacie on 27/12/2021.
//

#ifndef KMS_SLAM_IMUMANAGER_H
#define KMS_SLAM_IMUMANAGER_H

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/base/Matrix.h>

#include <mutex>
#include <deque>

#include "imu/IMUdata.h"

namespace kms_slam
{
    using namespace gtsam;

    class IMUManager
    {
    public:
        explicit IMUManager(boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams);

        void addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro);

        CombinedImuFactor getFactor(double startTime, double endTime, uint64_t currentIndex, imuBias::ConstantBias bias);
        CombinedImuFactor getFactor(double endTime, uint64_t currentIndex, imuBias::ConstantBias bias);
    protected:
        using LockGuard = std::lock_guard<std::mutex>;

        std::mutex _bufferMutex;
        std::deque<KMS_IMUData> _buffer;
        PreintegratedCombinedMeasurements _integrator;
    };
}

#endif //KMS_SLAM_IMUMANAGER_H
