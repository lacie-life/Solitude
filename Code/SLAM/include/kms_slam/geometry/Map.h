#ifndef MAP_H
#define MAP_H

#include "geometry/MapPoint.h"
#include "geometry/KeyFrame.h"
#include <set>

#include <mutex>


namespace kms_slam {

    class MapPoint;

    class KeyFrame;

    class KFIdComapre {
    public:
        bool operator()(const KeyFrame *kfleft, const KeyFrame *kfright) const;
    };

    class Map {
    public:
        // Update after an absolute scale is available
        void UpdateScale(const double &scale);

        //-----------------------------------------
    public:
        Map();

        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);

        void EraseMapPoint(MapPoint *pMP);

        void EraseKeyFrame(KeyFrame *pKF);

        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        std::vector<KeyFrame *> GetAllKeyFrames();

        std::vector<MapPoint *> GetAllMapPoints();

        std::vector<MapPoint *> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();

        long unsigned KeyFramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        vector<KeyFrame *> mvpKeyFrameOrigins;

        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

    protected:
        std::set<MapPoint *> mspMapPoints;
        std::set<KeyFrame *, KFIdComapre> mspKeyFrames;

        std::vector<MapPoint *> mvpReferenceMapPoints;

        long unsigned int mnMaxKFid;

        std::mutex mMutexMap;
    };

} //namespace kms_slam

#endif // MAP_H
