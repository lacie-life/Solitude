//
// Created by jun on 29/12/2021.
//

#ifndef KMS_SLAM_SMARTFACTORS_H
#define KMS_SLAM_SMARTFACTORS_H

#include <utility>
#include <boost/shared_ptr.hpp>

namespace gtsam
{
    class Values;
    class Pose3;
    class Point3;
    class Cal3_S2;

    template<typename KEY, typename VALUE> class FastMap;
    template<class CALIBRATION> class SmartProjectionPoseFactor;
    template<class POSE, class LANDMARK, class CALIBRATION>
    class GenericProjectionFactor;
    class SmartProjectionParams;
    template <class VALUE> class BetweenFactor;
    template<class Pose> class PoseBetweenFactor;
    template<class Pose, class Point, class T> class RangeFactorWithTransform;
    template<class POSE, class LANDMARK> class CameraBearingFactor;
    template<class POSE, class LANDMARK> class CameraBearingExtrinsicsFactor;
} // end namespace gtsam

namespace kms_slam
{
    // Typedef of desired classes to reduce compile-time
    using ProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;
    using SmartFactor = gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>;
    using SmartFactorPtr = boost::shared_ptr<SmartFactor>;
    using RelativePoseFactor = gtsam::PoseBetweenFactor<gtsam::Pose3>;
    using CamPointDistFactor = gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3, double>;
    using CameraBearingFactor3D = gtsam::CameraBearingFactor<gtsam::Pose3, gtsam::Point3>;
    using CameraBearingTbcFactor = gtsam::CameraBearingExtrinsicsFactor<gtsam::Pose3, gtsam::Point3>;
    using PointMatchFactor = gtsam::BetweenFactor<gtsam::Point3>;

    struct SmartFactorInfo
    {
        boost::shared_ptr<SmartFactor> factor_ = nullptr;
        int slot_in_graph_ = -1;
        SmartFactorInfo() = delete;
        SmartFactorInfo(const boost::shared_ptr<SmartFactor>& factor,
                        const int slot_idx)
                :factor_(factor), slot_in_graph_(slot_idx)
        {

        }
    };

    using SmartFactorInforMap = gtsam::FastMap<int, kms_slam::SmartFactorInfo>;
} // end namespace kms_slam

#endif //KMS_SLAM_SMARTFACTORS_H
