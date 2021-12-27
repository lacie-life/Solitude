#ifndef IMUDATA_H
#define IMUDATA_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/Matrix.h>

/**
* IMU model https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
* the web IMU Parameter[sigma_g, sigma_gw, sigma_a, sigma_aw]
* For EuRoc dataset, according to V1_01_easy/imu0/sensor.yaml
* The params:
* sigma_g: 1.6968e-4       rad / s / sqrt(Hz)    n_g(gyroscope_noise_density)           // continuous white noise for gyroscope
* sigma_gw: 1.9393e-5      rad / s^2 / sqrt(Hz)  n_bg(gyroscope_random_walk)            // continuous random walk for gyroscope
* sigma_a: 2.0e-3          m / s^2 / sqrt(Hz)    na(accelerometer_noise_density)        // continuous white noise for accelerometer
* sigma_aw: 3.0e-3         m / s^3 / sqrt(Hz)    n_ba(accelerometer_random_walk)        // continuous random walk for accelerometer
*/

namespace kms_slam {

    using namespace Eigen;

    class IMUData {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // covariance of measurement
        static Matrix3d _gyrMeasCov;
        static Matrix3d _accMeasCov;

        static Matrix3d getGyrMeasCov(void) { return _gyrMeasCov; }

        static Matrix3d getAccMeasCov(void) { return _accMeasCov; }

        // covariance of bias random walk
        static Matrix3d _gyrBiasRWCov;
        static Matrix3d _accBiasRWCov;

        static Matrix3d getGyrBiasRWCov(void) { return _gyrBiasRWCov; }

        static Matrix3d getAccBiasRWCov(void) { return _accBiasRWCov; }

        static double _gyrBiasRw2;
        static double _accBiasRw2;

        static double getGyrBiasRW2(void) { return _gyrBiasRw2; }

        static double getAccBiasRW2(void) { return _accBiasRw2; }


        IMUData(const double &gx, const double &gy, const double &gz,
                const double &ax, const double &ay, const double &az,
                const double &t);
        //IMUData(const IMUData& imu);

        // Raw data of imu's
        Vector3d _g;    //gyr data
        Vector3d _a;    //acc data
        double _t;      //timestamp
    };

    class KMS_IMUData{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KMS_IMUData() = default;

        KMS_IMUData(const double &gx, const double &gy, const double &gz,
                    const double &ax, const double &ay, const double &az,
                    const double &t);

        KMS_IMUData(const double time, const gtsam::Vector3 accel, const gtsam::Vector3 gyro);

        //IMU raw data
        Eigen::Vector3d gyroscope_;
        Eigen::Vector3d accelerometer_;

        //Data timestamp
        double timestamp_;

        //IMU sensor parameters, no need to set multiple times
        //IMU sensor continuous Gaussian white noise
        static double sigma_gyroscope_;
        static double sigma_accelerometer_;

        //IMU sensor continous random walk
        static double sigma_gyroscope_walk_;
        static double sigma_accelerometer_walk_;

        //IMU sensor data collection interval
        static double delta_timestamp_;

        //For the correspondences between discrete and continuous parameters, see the introduction of IMU model
        //Square of discrete random walk of IMU sensor
        static double gyroscope_noise_rw2_;
        static double accelerometer_noise_rw2_;

        //Square of discrete noise of IMU data
        static double gyroscope_bias_rw2_;
        static double accelerometer_bias_rw2_;

        //Noise matrix
        static Eigen::Matrix3d gyroscope_meas_covariance_;
        static Eigen::Matrix3d accelerometer_meas_covariance_;

        //random walk matrix
        static Eigen::Matrix3d gyroscope_bias_rw_covariance_;
        static Eigen::Matrix3d accelerometer_bias_rw_covariance_;
    };

} // namespace kms_slam

#endif // IMUDATA_H
