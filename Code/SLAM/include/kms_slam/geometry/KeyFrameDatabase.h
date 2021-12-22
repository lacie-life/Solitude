
#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "geometry/KeyFrame.h"
#include "geometry/Frame.h"
#include "feature/ORBVocabulary.h"

#include <mutex>


namespace kms_slam {

    class KeyFrame;

    class Frame;


    class KeyFrameDatabase {
    public:

        KeyFrameDatabase(const ORBVocabulary &voc);

        void add(KeyFrame *pKF);

        void erase(KeyFrame *pKF);

        void clear();

        // Loop Detection
        std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

        // Relocalization
        std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);

    protected:

        // Associated vocabulary
        const ORBVocabulary *mpVoc;

        // Inverted file
        std::vector<list<KeyFrame *> > mvInvertedFile;

        // Mutex
        std::mutex mMutex;
    };

} //namespace kms_slam

#endif
