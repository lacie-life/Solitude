//
// Created by jun on 29/12/2021.
//

#ifndef KMS_SLAM_TRANSFORMATION_H
#define KMS_SLAM_TRANSFORMATION_H

#include "Thirdparty/minkindr/include/quat-transformation.h"

namespace kms_slam
{
    using Transformation = kindr::minimal::QuatTransformation;
    using Quaternion = kindr::minimal::RotationQuaternion;
}

#endif //KMS_SLAM_TRANSFORMATION_H
