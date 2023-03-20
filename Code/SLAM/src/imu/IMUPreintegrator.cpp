#include "imu/IMUPreintegrator.h"

namespace kms_slam {

    IMUPreintegrator::IMUPreintegrator(const IMUPreintegrator &pre) :
            _delta_P(pre._delta_P),
            _delta_V(pre._delta_V),
            _delta_R(pre._delta_R),
            _J_P_Biasg(pre._J_P_Biasg),
            _J_P_Biasa(pre._J_P_Biasa),
            _J_V_Biasg(pre._J_V_Biasg),
            _J_V_Biasa(pre._J_V_Biasa),
            _J_R_Biasg(pre._J_R_Biasg),
            _cov_P_V_Phi(pre._cov_P_V_Phi),
            _delta_time(pre._delta_time) {

    }


    //IMUPreintegrator& IMUPreintegrator::operator = (const IMUPreintegrator& pre)
    //{
    //    _delta_P = pre._delta_P;
    //    _delta_V = pre._delta_V;
    //    _delta_R = pre._delta_R;
    //    _J_P_Biasg = pre._J_P_Biasg;
    //    _J_P_Biasa = pre._J_P_Biasa;
    //    _J_V_Biasg = pre._J_V_Biasg;
    //    _J_V_Biasa = pre._J_V_Biasa;
    //    _J_R_Biasg = pre._J_R_Biasg;
    //    _cov_P_V_Phi = pre._cov_P_V_Phi;
    //    _delta_time = pre._delta_time;

    //}

    IMUPreintegrator::IMUPreintegrator() {
        // delta measurements, position/velocity/rotation(matrix)
        _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
        _delta_V.setZero();    // V_k+1 = V_k + R_k*a_k*dt
        _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

        // jacobian of delta measurements w.r.t bias of gyro/acc
        _J_P_Biasg.setZero();     // position / gyro
        _J_P_Biasa.setZero();     // position / acc
        _J_V_Biasg.setZero();     // velocity / gyro
        _J_V_Biasa.setZero();     // velocity / acc
        _J_R_Biasg.setZero();   // rotation / gyro

        // noise covariance propagation of delta measurements
        _cov_P_V_Phi.setZero();

        _delta_time = 0;
    }

    void IMUPreintegrator::reset() {
        // delta measurements, position/velocity/rotation(matrix)
        _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
        _delta_V.setZero();    // V_k+1 = V_k + R_k*a_k*dt
        _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

        // jacobian of delta measurements w.r.t bias of gyro/acc
        _J_P_Biasg.setZero();     // position / gyro
        _J_P_Biasa.setZero();     // position / acc
        _J_V_Biasg.setZero();     // velocity / gyro
        _J_V_Biasa.setZero();     // velocity / acc
        _J_R_Biasg.setZero();   // rotation / gyro

        // noise covariance propagation of delta measurements
        _cov_P_V_Phi.setZero();

        _delta_time = 0;

    }

    // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
    // acc: acc_measurement - bias_a, last measurement!! not current measurement
    // omega: gyro_measurement - bias_g, last measurement!! not current measurement
    void IMUPreintegrator::update(const Vector3d &omega, const Vector3d &acc, const double &dt) {
        double dt2 = dt * dt;

        Matrix3d dR = Expmap(omega * dt);
        Matrix3d Jr = JacobianR(omega * dt);

        // noise covariance propagation of delta measurements
        // err_k+1 = A*err_k + B*err_gyro + C*err_acc
        Matrix3d I3x3 = Matrix3d::Identity();
        Matrix<double, 9, 9> A = Matrix<double, 9, 9>::Identity();
        A.block<3, 3>(6, 6) = dR.transpose();
        A.block<3, 3>(3, 6) = -_delta_R * skew(acc) * dt;
        A.block<3, 3>(0, 6) = -0.5 * _delta_R * skew(acc) * dt2;
        A.block<3, 3>(0, 3) = I3x3 * dt;
        Matrix<double, 9, 3> Bg = Matrix<double, 9, 3>::Zero();
        Bg.block<3, 3>(6, 0) = Jr * dt;
        Matrix<double, 9, 3> Ca = Matrix<double, 9, 3>::Zero();
        Ca.block<3, 3>(3, 0) = _delta_R * dt;
        Ca.block<3, 3>(0, 0) = 0.5 * _delta_R * dt2;
        _cov_P_V_Phi = A * _cov_P_V_Phi * A.transpose() +
                       Bg * IMUData::getGyrMeasCov() * Bg.transpose() +
                       Ca * IMUData::getAccMeasCov() * Ca.transpose();


        // jacobian of delta measurements w.r.t bias of gyro/acc
        // update P first, then V, then R
        _J_P_Biasa += _J_V_Biasa * dt - 0.5 * _delta_R * dt2;
        _J_P_Biasg += _J_V_Biasg * dt - 0.5 * _delta_R * skew(acc) * _J_R_Biasg * dt2;
        _J_V_Biasa += -_delta_R * dt;
        _J_V_Biasg += -_delta_R * skew(acc) * _J_R_Biasg * dt;
        _J_R_Biasg = dR.transpose() * _J_R_Biasg - Jr * dt;

        // delta measurements, position/velocity/rotation(matrix)
        // update P first, then V, then R. because P's update need V&R's previous state
        _delta_P += _delta_V * dt + 0.5 * _delta_R * acc * dt2;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
        _delta_V += _delta_R * acc * dt;
        _delta_R = normalizeRotationM(_delta_R * dR);  // normalize rotation, in case of numerical error accumulation


    //    // noise covariance propagation of delta measurements
    //    // err_k+1 = A*err_k + B*err_gyro + C*err_acc
    //    Matrix3d I3x3 = Matrix3d::Identity();
    //    MatrixXd A = MatrixXd::Identity(9,9);
    //    A.block<3,3>(6,6) = dR.transpose();
    //    A.block<3,3>(3,6) = -_delta_R*skew(acc)*dt;
    //    A.block<3,3>(0,6) = -0.5*_delta_R*skew(acc)*dt2;
    //    A.block<3,3>(0,3) = I3x3*dt;
    //    MatrixXd Bg = MatrixXd::Zero(9,3);
    //    Bg.block<3,3>(6,0) = Jr*dt;
    //    MatrixXd Ca = MatrixXd::Zero(9,3);
    //    Ca.block<3,3>(3,0) = _delta_R*dt;
    //    Ca.block<3,3>(0,0) = 0.5*_delta_R*dt2;
    //    _cov_P_V_Phi = A*_cov_P_V_Phi*A.transpose() +
    //        Bg*IMUData::getGyrMeasCov*Bg.transpose() +
    //        Ca*IMUData::getAccMeasCov()*Ca.transpose();

        // delta time
        _delta_time += dt;

    }

    // ---------------------------KMS_IMUPreintegrator class----------------------//
    KMS_IMUPreintegrator::KMS_IMUPreintegrator(){
        delta_p_.setZero();
        delta_v_.setZero();
        delta_rot_.setIdentity();

        jacobian_v_bias_gyroscope_.setZero();
        jacobian_v_bias_accelerometer_.setZero();

        jacobian_p_bias_gyroscope_.setZero();
        jacobian_p_bias_accelerometer_.setZero();

        jacobian_rot_bias_gyroscope_.setZero();

        cov_p_v_rot_.setZero();

        delta_time_ = 0.0;
    }

    KMS_IMUPreintegrator::KMS_IMUPreintegrator(const KMS_IMUPreintegrator &preintegrator) :
            delta_p_(preintegrator.delta_p_), delta_v_(preintegrator.delta_v_), delta_rot_(preintegrator.delta_rot_),
            jacobian_p_bias_gyroscope_(),
            jacobian_p_bias_accelerometer_(),
            jacobian_v_bias_gyroscope_(),
            jacobian_v_bias_accelerometer_(),
            jacobian_rot_bias_gyroscope_(),
            cov_p_v_rot_(), delta_time_(preintegrator.delta_time_) {}

    void KMS_IMUPreintegrator::Update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc,const double &delta_timestamp_){
        double dt2 = delta_timestamp_*delta_timestamp_;

        //Step-1: Calculate IMU pre-integral rotation increment and Jacobian
        Eigen::Matrix3d dR = Sophus::SO3::exp(omega*delta_timestamp_).matrix();
        Eigen::Matrix3d Jr_R = Sophus::SO3::JacobianR(omega*delta_timestamp_);

        //Step 2: Calculate the uncertainity variance of the noise error of the IMU pre-integrator observation equation
        Eigen::Matrix3d I3x3;
        I3x3.setIdentity();

        Matrix9d A;
        A.setIdentity();
        //  The block size is 3x3, the starting position (6,6)
        A.block<3,3>(6,6) = dR.transpose();
        A.block<3,3>(3,6) = -delta_rot_ * Sophus::SO3::hat(acc) * delta_timestamp_;
        A.block<3,3>(0,6) = -0.5* delta_rot_ *Sophus::SO3::hat(acc) * dt2;
        A.block<3,3>(0,3) = I3x3 *delta_timestamp_;

        //Split the B in the paper into two parts
        Eigen::Matrix<double,9,3> Bg;
        Bg.setZero();
        Bg.block<3,3>(6,0) = Jr_R * delta_timestamp_;

        Eigen::Matrix<double,9,3> Ca;
        Ca.setZero();
        Ca.block<3,3>(3,0) = delta_rot_ * delta_timestamp_;
        Ca.block<3,3>(0,0) = 0.5 * delta_rot_ * dt2;

        cov_p_v_rot_ = A * cov_p_v_rot_ * A.transpose() +
                       Bg * KMS_IMUData::gyroscope_meas_covariance_ * Bg.transpose() +
                       Ca * KMS_IMUData::accelerometer_meas_covariance_ * Ca.transpose();

        //Step-3: Calculate Jacobian, used to correct the biased IMU pre-integrated measurement value

        //Recursion based on supplementary material formula
        jacobian_p_bias_accelerometer_ += jacobian_v_bias_accelerometer_ * delta_timestamp_ -0.5 * delta_rot_ * dt2;
        jacobian_p_bias_gyroscope_ += jacobian_v_bias_gyroscope_ * delta_timestamp_ - 0.5 * delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyroscope_ * dt2;

        jacobian_v_bias_accelerometer_ += -delta_rot_ * delta_timestamp_;
        jacobian_v_bias_gyroscope_ += -delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyroscope_ * delta_timestamp_;

        jacobian_rot_bias_gyroscope_ = dR.transpose() * jacobian_rot_bias_gyroscope_ - Jr_R * delta_timestamp_;

        //Step-4: Calculate IMU pre-integrated measurement value
        delta_p_ += delta_v_*delta_timestamp_ + 0.5 * delta_rot_ * acc * dt2;
        delta_v_ += delta_rot_ * acc * delta_timestamp_;
        delta_rot_ += NormalizedRotationM(delta_rot_*dR);
        delta_time_ += delta_timestamp_;
    }

    void KMS_IMUPreintegrator::Reset(){
        delta_p_.setZero();
        delta_v_.setZero();
        delta_rot_.setIdentity();

        jacobian_v_bias_gyroscope_.setZero();
        jacobian_v_bias_accelerometer_.setZero();

        jacobian_p_bias_gyroscope_.setZero();
        jacobian_p_bias_accelerometer_.setZero();

        jacobian_rot_bias_gyroscope_.setZero();

        cov_p_v_rot_.setZero();

        delta_time_ = 0.0;
    }

} // namespace kms_slam
