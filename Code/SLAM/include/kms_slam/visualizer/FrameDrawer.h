#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "system/Tracking.h"
#include "geometry/MapPoint.h"
#include "geometry/Map.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>


namespace kms_slam {

    class Tracking;

    class Viewer;

    class FrameDrawer {
    public:
        FrameDrawer(Map *pMap);

        // Update info from the last processed frame.
        void Update(Tracking *pTracker);

        // Draw last processed frame.
        cv::Mat DrawFrame();

    protected:
        double mStartTime;
        double mCurTime;

        void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

        // Info of the frame to be drawn
        cv::Mat mIm;
        int N;
        vector <cv::KeyPoint> mvCurrentKeys;
        vector<bool> mvbMap, mvbVO;
        bool mbOnlyTracking;
        int mnTracked, mnTrackedVO;
        vector <cv::KeyPoint> mvIniKeys;
        vector<int> mvIniMatches;
        int mState;

        Map *mpMap;

        std::mutex mMutex;
    };

} //namespace kms_slam

#endif // FRAMEDRAWER_H
