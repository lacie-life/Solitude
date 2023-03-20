
#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

namespace kms_slam {

    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
            ORBVocabulary;

} //namespace kms_slam

#endif // ORBVOCABULARY_H
