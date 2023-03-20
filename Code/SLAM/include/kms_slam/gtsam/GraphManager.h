//
// Created by lacie on 27/12/2021.
//

#ifndef KMS_SLAM_GRAPHMANAGER_H
#define KMS_SLAM_GRAPHMANAGER_H

#include "gtsam/IMUManager.h"
#include "gtsam/SmartFactors.h"
#include "common/Types.h"
#include "common/Transformation.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/geometry/Cal3_S2.h>

#include <mutex>
#include <atomic>
#include <tuple>
#include <map>
#include <memory>
#include <thread>
#include <vector>
#include <unordered_map>
#include <fstream>

using namespace gtsam;

namespace kms_slam
{
    struct ImuCalibration;
    struct ImuInitialization;
    struct ImuMeasurement;
    typedef std::deque<ImuMeasurement, Eigen::aligned_allocator<ImuMeasurement>>
            ImuMeasurements;
    class Point;
    class Frame;
    using GTSAMSlotIndices = gtsam::FastVector<size_t>;

    struct GraphManagerOption
    {
        // noise of visual measurements in pixel
        double reproj_error_ns_px = 1.0;
        double smart_reproj_outlier_thresh_px = 3.0;
        bool use_robust_px_noise = true;

        // this avoids chierality check
        bool use_bearing_factor = true;

        // prior for visual-inertial case
        double init_pos_sigma_meter = 0.001;
        double init_roll_pitch_sigma_rad = 45.0 / 180.0 * M_PI;
        double init_yaw_sigma_rad = 1.0 / 180.0 * M_PI;

        // default general pose prior
        double position_prior_sigma_meter = 0.001;
        double rotation_prior_sigma_rad = 1.0 / 180 * M_PI;
        double point_prior_sigma_meter = 0.001;
    };

    struct SmartFactorStatistics
    {
        size_t n_valid = 0;
        size_t n_degenerate = 0;
        size_t n_behind_cam = 0;
        size_t n_outlier = 0;
        size_t n_farpoint = 0;
        void print() const;
    };

    class GraphManager
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using NFramesId = int;
        using PointId = int;
        using Ptr = std::shared_ptr<GraphManager>;
        using PreintegratedImuMeasurements = gtsam::PreintegratedCombinedMeasurements;
        using CombinedPreintegratedMeasurementPtr =
        std::shared_ptr<PreintegratedImuMeasurements>;
        using PointIdSmartFactorMap =
        std::unordered_map<int, boost::shared_ptr<SmartFactor>>;
        using IdToFactorIdxMap = std::map<int, std::vector<size_t>>;
        using IdToSlotIdMap = std::map<int, std::vector<int>>;

        GraphManager(const GraphManagerOption& options);
        ~GraphManager();

        GraphManagerOption options_;
        gtsam::Cal3_S2::shared_ptr cam_calib;
        boost::shared_ptr<PreintegratedImuMeasurements::Params> preintegration_params_;
        std::vector<gtsam::SharedNoiseModel> uplane_noise_pyr_;
        gtsam::SharedNoiseModel smart_noise_;
        gtsam::SharedNoiseModel imu_bias_prior_noise_;
        gtsam::SharedNoiseModel velocity_prior_noise_;
        gtsam::SharedNoiseModel zero_velocity_prior_noise_;
        gtsam::SharedNoiseModel point_match_noise_;
        Eigen::Matrix<double, 6, 6> init_pose_prior_visual_inertial_;
        std::unique_ptr<gtsam::SmartProjectionParams> smart_factor_params_;

        // Mutex protected
        std::mutex graph_mut_;
        gtsam::NonlinearFactorGraph new_factors_; // new factors to be added
        gtsam::Values new_values_;                // new states to be added

        // Smart factors
        PointIdSmartFactorMap point_id_to_new_smart_factor_map_; // pointId -> {SmartFactorPtr}
        SmartFactorInforMap point_id_to_smart_factor_infor_map_; // pointId -> {SmartFactorPtr, SlotIndex}

        SmartFactorStatistics smart_factor_stats_;




    }; // class GraphManager
}

#endif //KMS_SLAM_GRAPHMANAGER_H
