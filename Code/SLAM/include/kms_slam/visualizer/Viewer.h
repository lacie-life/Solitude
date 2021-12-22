
#ifndef VIEWER_H
#define VIEWER_H

#include "visualizer/FrameDrawer.h"
#include "visualizer/MapDrawer.h"
#include "system/Tracking.h"
#include "system/System.h"

#include <mutex>

namespace kms_slam {

    class Tracking;

    class FrameDrawer;

    class MapDrawer;

    class System;

    class Viewer {
    public:
        Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
               const string &strSettingPath);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

    private:

        bool Stop();

        System *mpSystem;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        Tracking *mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

    };

} //namespace kms_slam

#endif // VIEWER_H
	

