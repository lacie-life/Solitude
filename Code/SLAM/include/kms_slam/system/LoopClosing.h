
#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "geometry/KeyFrame.h"
#include "geometry/Map.h"
#include "geometry/KeyFrameDatabase.h"

#include "system/LocalMapping.h"
#include "system/Tracking.h"

#include "feature/ORBVocabulary.h"

#include "common/Configparam.h"

#include "g2o/types/sim3/types_seven_dof_expmap.h"

#include <thread>
#include <mutex>

namespace kms_slam {

    class Tracking;

    class LocalMapping;

    class KeyFrameDatabase;


    class LoopClosing {
    public:
        ConfigParam *mpParams;

        bool GetMapUpdateFlagForTracking();

        void SetMapUpdateFlagInTracking(bool bflag);

    protected:
        std::mutex mMutexMapUpdateFlag;
        bool mbMapUpdateFlagForTracking;

    public:

        typedef pair<set<KeyFrame *>, int> ConsistentGroup;
        typedef map<KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
                Eigen::aligned_allocator<std::pair<KeyFrame * const, g2o::Sim3> > > KeyFrameAndPose;

    public:

        LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, ConfigParam *pParams);

        void SetTracker(Tracking *pTracker);

        void SetLocalMapper(LocalMapping *pLocalMapper);

        // Main function
        void Run();

        void InsertKeyFrame(KeyFrame *pKF);

        void RequestReset();

        // This function will run in a separate thread
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);

        bool isRunningGBA() {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbRunningGBA;
        }

        bool isFinishedGBA() {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbFinishedGBA;
        }

        void RequestFinish();

        bool isFinished();

    protected:

        bool CheckNewKeyFrames();

        bool DetectLoop();

        bool ComputeSim3();

        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

        void CorrectLoop();

        void ResetIfRequested();

        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Map *mpMap;
        Tracking *mpTracker;

        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBVocabulary;

        LocalMapping *mpLocalMapper;

        std::list<KeyFrame *> mlpLoopKeyFrameQueue;

        std::mutex mMutexLoopQueue;

        // Loop detector parameters
        float mnCovisibilityConsistencyTh;

        // Loop detector variables
        KeyFrame *mpCurrentKF;
        KeyFrame *mpMatchedKF;
        std::vector<ConsistentGroup> mvConsistentGroups;
        std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
        std::vector<KeyFrame *> mvpCurrentConnectedKFs;
        std::vector<MapPoint *> mvpCurrentMatchedPoints;
        std::vector<MapPoint *> mvpLoopMapPoints;
        cv::Mat mScw;
        g2o::Sim3 mg2oScw;

        long unsigned int mLastLoopKFid;

        // Variables related to Global Bundle Adjustment
        bool mbRunningGBA;
        bool mbFinishedGBA;
        bool mbStopGBA;
        std::mutex mMutexGBA;
        std::thread *mpThreadGBA;

        // Fix scale in the stereo/RGB-D case
        bool mbFixScale;

        bool mnFullBAIdx;
    };

} //namespace kms_slam

#endif // LOOPCLOSING_H
