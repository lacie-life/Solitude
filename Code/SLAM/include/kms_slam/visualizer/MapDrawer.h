
#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "geometry/Map.h"
#include "geometry/MapPoint.h"
#include "geometry/KeyFrame.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace kms_slam {

    class MapDrawer {
    public:
        MapDrawer(Map *pMap, const string &strSettingPath);

        Map *mpMap;

        void DrawMapPoints();

        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

        void SetCurrentCameraPose(const cv::Mat &Tcw);

        void SetReferenceKeyFrame(KeyFrame *pKF);

        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    private:

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        cv::Mat mCameraPose;

        std::mutex mMutexCamera;
    };

} //namespace kms_slam

#endif // MAPDRAWER_H
