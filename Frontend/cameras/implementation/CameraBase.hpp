/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Feb 3, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/CameraBase.hpp
 * @brief Header implementation file for the CameraBase class.
 * @author Stefan Leutenegger
 */

#include <iostream>

/// \brief vio Main namespace of this package.
namespace vio {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// Set the mask. It must be the same size as the image and
bool CameraBase::setMask(const cv::Mat & mask) {
  // check type
  if (mask.type() != CV_8UC1) {
    return false;
  }
  // check size
  if (mask.rows != imageHeight_) {
    return false;
  }
  if (mask.cols != imageWidth_) {
    return false;
  }
  mask_ = mask;
  return true;
}

/// Was a nonzero mask set?
bool CameraBase::removeMask() {
  mask_.resize(0);
  return true;
}

// Was acc nonzero mask set?
bool CameraBase::hasMask() const {
  return (mask_.data);
}

// Get the mask.
const cv::Mat & CameraBase::mask() const {
  return mask_;
}
//这里的mask在eth原本的代码里应该是要求的中间是黑色,鱼眼外的是白色,但是现在是反的
bool CameraBase::isMasked(const Eigen::Vector2d& imagePoint) const {
  if (!isInImage(imagePoint)) {
    return true;
  }
  if (!hasMask()) {
    return false;
  }
  return !(mask_.at<uchar>(static_cast<int>(imagePoint[1]), static_cast<int>(imagePoint[0])));//这里mask的颜色相反的
}
bool CameraBase::isMasked(const Eigen::Vector2f& imagePoint) const {
  if (!isInImage(imagePoint)) {
    return true;
  }
  if (!hasMask()) {
    return false;
  }
  return !(mask_.at<uchar>(static_cast<int>(imagePoint[1]), static_cast<int>(imagePoint[0])));//这里mask的颜色相反的
}

// Check if the keypoint is in the image.
bool CameraBase::isInImage(const Eigen::Vector2d& imagePoint) const {
  if (imagePoint[0] < 0.0 || imagePoint[1] < 0.0) {
    return false;
  }
  if (imagePoint[0] >= imageWidth_ || imagePoint[1] >= imageHeight_) {
    return false;
  }
  return true;
}
bool CameraBase::isInImage(const Eigen::Vector2f& imagePoint) const {
  if (imagePoint[0] < 0.0 || imagePoint[1] < 0.0) {
    return false;
  }
  if (imagePoint[0] >= imageWidth_ || imagePoint[1] >= imageHeight_) {
    return false;
  }
  return true;
}
#ifdef __ARM_NEON__
/******************************************************************************
 * ipts               ->   layout: u0, u1, u2, u3, v0, v1, v2, v3
 * status             ->   0 for in image range, or 0xffffffff for out of range
 */
void CameraBase::isInImage(const float* ipts, unsigned int status[4]) const {
  uint32x4_t vres0, vres1;
  float32x4_t img_us = vld1q_f32(ipts);
  float32x4_t img_vs = vld1q_f32(ipts + 4);
  vres0 = vcltq_f32(img_us, vmovq_n_f32(0.f));
  vres1 = vcltq_f32(img_vs, vmovq_n_f32(0.f));
  vres0 = vorrq_u32(vcgeq_f32(img_us, vmovq_n_f32(imageWidth_)), vres0);
  vres1 = vorrq_u32(vcgeq_f32(img_vs, vmovq_n_f32(imageHeight_)), vres1);
  vres0 = vorrq_u32(vres0, vres1);
  vst1q_u32(status, vres0);
}
#endif

}  // namespace cameras
}  // namespace vio

