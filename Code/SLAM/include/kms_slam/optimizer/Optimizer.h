
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "geometry/Map.h"
#include "geometry/MapPoint.h"
#include "geometry/KeyFrame.h"
#include "geometry/Frame.h"

#include "system/LoopClosing.h"

#include "g2o/types/types_seven_dof_expmap.h"

namespace kms_slam {

    class LoopClosing;

    class Optimizer {
    public:
        void static GlobalBundleAdjustmentNavState(Map *pMap, const cv::Mat &gw, int nIterations, bool *pbStopFlag,
                                                   const unsigned long nLoopKF, const bool bRobust);

        int static
        PoseOptimization(Frame *pFrame, KeyFrame *pLastKF, const IMUPreintegrator &imupreint, const cv::Mat &gw,
                         const bool &bComputeMarg = false);

        int static
        PoseOptimization(Frame *pFrame, Frame *pLastFrame, const IMUPreintegrator &imupreint, const cv::Mat &gw,
                         const bool &bComputeMarg = false);

        void static
        LocalBundleAdjustmentNavState(KeyFrame *pKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag,
                                      Map *pMap, cv::Mat &gw, LocalMapping *pLM = NULL);

        Vector3d static OptimizeInitialGyroBias(const std::list<KeyFrame *> &lLocalKeyFrames);

        Vector3d static OptimizeInitialGyroBias(const std::vector<KeyFrame *> &vLocalKeyFrames);

        Vector3d static OptimizeInitialGyroBias(const std::vector<Frame> &vFrames);

        void static
        LocalBundleAdjustment(KeyFrame *pKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag, Map *pMap,
                              LocalMapping *pLM = NULL);

    public:
        void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP,
                                     int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);

        void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                           const unsigned long nLoopKF = 0, const bool bRobust = true);

        void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, LocalMapping *pLM = NULL);

        int static PoseOptimization(Frame *pFrame);

        // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
        void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                           const bool &bFixScale, LoopClosing *pLC = NULL);

        // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
        static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
                                g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
    };

} //namespace kms_slam

#endif // OPTIMIZER_H
