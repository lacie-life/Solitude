//
// Created by lacie on 27/12/2021.
//

#include "gtsam/IMUManager.h"
#include <gtsam/inference/Symbol.h>

namespace kms_slam {
    using gtsam::symbol_shorthand::X; // Pose
    using gtsam::symbol_shorthand::V; // Velocity
    using gtsam::symbol_shorthand::B; // IMU Bias

    IMUManager::IMUManager(boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams) :
            _integrator(imuParams, imuBias::ConstantBias(Vector6::Zero())) {
        _integrator.resetIntegration();
    }

    void IMUManager::addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro) {
        LockGuard guard(_bufferMutex);
        _buffer.push_back(KMS_IMUData(time, accel, gyro));
    }

    CombinedImuFactor
    IMUManager::getFactor(double startTime, double endTime, uint64_t currentIndex, imuBias::ConstantBias bias) {
        LockGuard _lockGuard(_bufferMutex);

//        std::cout << "######## IMU retrieving factor between " << startTime << " and " << endTime << " for index " << currentIndex << std::endl;

        KMS_IMUData prevMeas;
        // Get rid of any old measurements. Should be only 1, in theory.
        while (!_buffer.empty() && _buffer.front().timestamp_ <= startTime) {
            prevMeas = _buffer.front();
//            std::cout << "Skipping measurement at t=" << prevMeas.time << std::endl;
            _buffer.pop_front();
        }

        _integrator.resetIntegrationAndSetBias(bias);

        prevMeas.timestamp_ = startTime;
        // Integrate all of the easy measurements (that don't require interpolation).
        while (!_buffer.empty() && _buffer.front().timestamp_ < endTime) {
            auto newMeas = _buffer.front();
            _buffer.pop_front();
            _integrator.integrateMeasurement(
                    newMeas.accelerometer_, newMeas.gyroscope_, newMeas.timestamp_ - prevMeas.timestamp_
            );
            prevMeas = newMeas;
        }

        // The final measurement requires interpolation.
        if (!_buffer.empty()) {
            auto finalMeas = _buffer.front();
            const double interpolationFactor = (endTime - prevMeas.timestamp_) / (finalMeas.timestamp_ - prevMeas.timestamp_);
            const Vector3 interpAccel =
                    (interpolationFactor * finalMeas.accelerometer_) + ((1.0 - interpolationFactor) * prevMeas.accelerometer_);
            const Vector3 interpGyro =
                    (interpolationFactor * finalMeas.gyroscope_) + ((1.0 - interpolationFactor) * prevMeas.gyroscope_);

            _integrator.integrateMeasurement(interpAccel, interpGyro, endTime - prevMeas.timestamp_);

        }

        return CombinedImuFactor(
                X(currentIndex - 1), V(currentIndex - 1),
                X(currentIndex), V(currentIndex),
                B(currentIndex - 1), B(currentIndex),
                _integrator
        );
    }

    CombinedImuFactor IMUManager::getFactor(double endTime, uint64_t currentIndex, imuBias::ConstantBias bias) {
        return getFactor(_buffer.front().timestamp_, endTime, currentIndex, bias);
    }
}
