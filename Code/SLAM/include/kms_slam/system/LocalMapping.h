
#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "common/Configparam.h"

#include "geometry/KeyFrame.h"
#include "geometry/Map.h"
#include "geometry/KeyFrameDatabase.h"

#include "system/LoopClosing.h"
#include "system/Tracking.h"

#include <mutex>

namespace kms_slam {

    class Tracking;

    class LoopClosing;

    class Map;

    class LocalMapping {
    public:
        ConfigParam *mpParams;

        // KeyFrames in Local Window, for Local BA
        // Insert in ProcessNewKeyFrame()
        void AddToLocalWindow(KeyFrame *pKF);

        void DeleteBadInLocalWindow(void);

        bool TryInitVIO(void);

        bool TryInitStereoVIO(void);

        bool GetVINSInited(void);

        void SetVINSInited(bool flag);

        bool GetFirstVINSInited(void);

        void SetFirstVINSInited(bool flag);

        cv::Mat GetGravityVec(void);

        bool GetMapUpdateFlagForTracking();

        void SetMapUpdateFlagInTracking(bool bflag);

        KeyFrame *GetMapUpdateKF();

    protected:
        double mnStartTime;
        bool mbFirstTry;
        double mnVINSInitScale;
        cv::Mat mGravityVec; // gravity vector in world frame

        std::mutex mMutexVINSInitFlag;
        bool mbVINSInited;

        std::mutex mMutexFirstVINSInitFlag;
        bool mbFirstVINSInited;

        unsigned int mnLocalWindowSize;
        std::list<KeyFrame *> mlLocalKeyFrames;

        std::mutex mMutexMapUpdateFlag;
        bool mbMapUpdateFlagForTracking;
        KeyFrame *mpMapUpdateKF;

    public:
        LocalMapping(Map *pMap, const float bMonocular, ConfigParam *pParams);

        void SetLoopCloser(LoopClosing *pLoopCloser);

        void SetTracker(Tracking *pTracker);

        // Main function
        void Run();

        void InsertKeyFrame(KeyFrame *pKF);

        // Thread Synch
        void RequestStop();

        void RequestReset();

        bool Stop();

        void Release();

        bool isStopped();

        bool stopRequested();

        bool AcceptKeyFrames();

        void SetAcceptKeyFrames(bool flag);

        bool SetNotStop(bool flag);

        void InterruptBA();

        void RequestFinish();

        bool isFinished();

        int KeyframesInQueue() {
            unique_lock<std::mutex> lock(mMutexNewKFs);
            return mlNewKeyFrames.size();
        }

    protected:

        bool CheckNewKeyFrames();

        void ProcessNewKeyFrame();

        void CreateNewMapPoints();

        void MapPointCulling();

        void SearchInNeighbors();

        void KeyFrameCulling();

        cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

        cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

        bool mbMonocular;

        void ResetIfRequested();

        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Map *mpMap;

        LoopClosing *mpLoopCloser;
        Tracking *mpTracker;

        std::list<KeyFrame *> mlNewKeyFrames;

        KeyFrame *mpCurrentKeyFrame;

        std::list<MapPoint *> mlpRecentAddedMapPoints;

        std::mutex mMutexNewKFs;

        bool mbAbortBA;

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;

        bool mbAcceptKeyFrames;
        std::mutex mMutexAccept;
    };

} //namespace kms_slam

#endif // LOCALMAPPING_H
