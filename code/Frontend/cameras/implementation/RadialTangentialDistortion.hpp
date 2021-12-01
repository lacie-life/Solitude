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
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */


#include <Eigen/LU>
#include <iostream>

/// \brief vio Main namespace of this package.
namespace vio {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// The default constructor with all zero ki
RadialTangentialDistortion::RadialTangentialDistortion()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0) {
  parameters_.setZero();
}

// Constructor initialising ki
RadialTangentialDistortion::RadialTangentialDistortion(float k1, float k2,
                                                       float p1, float p2) {
  parameters_[0] = k1;
  parameters_[1] = k2;
  parameters_[2] = p1;
  parameters_[3] = p2;
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
}

bool RadialTangentialDistortion::setParameters(
    const Eigen::VectorXd & parameters) {
  if (parameters.cols() != NumDistortionIntrinsics) {
    return false;
  }
  parameters_ = parameters.cast<float>();
  k1_ = parameters[0];
  k2_ = parameters[1];
  p1_ = parameters[2];
  p2_ = parameters[3];
  return true;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted,
    Eigen::Vector2d * pointDistorted) const {
  // just compute the distorted point
  const float u0 = pointUndistorted[0];
  const float u1 = pointUndistorted[1];
  const float mx_u = u0 * u0;
  const float my_u = u1 * u1;
  const float mxy_u = u0 * u1;
  const float rho_u = mx_u + my_u;
  const float rad_dist_u = k1_ * rho_u + k2_ * rho_u * rho_u;
  (*pointDistorted)[0] = u0 + u0 * rad_dist_u + 2.0 * p1_ * mxy_u
      + p2_ * (rho_u + 2.0 * mx_u);
  (*pointDistorted)[1] = u1 + u1 * rad_dist_u + 2.0 * p2_ * mxy_u
      + p1_ * (rho_u + 2.0 * my_u);
  return true;
}
bool RadialTangentialDistortion::distort(const Eigen::Vector2f & pointUndistorted,
                                         Eigen::Vector2f * pointDistorted) const {
  // just compute the distorted point
  const float u0 = pointUndistorted[0];
  const float u1 = pointUndistorted[1];
  const float mx_u = u0 * u0;
  const float my_u = u1 * u1;
  const float mxy_u = u0 * u1;
  const float rho_u = mx_u + my_u;
  const float rad_dist_u = k1_ * rho_u + k2_ * rho_u * rho_u;
  (*pointDistorted)[0] = u0 + u0 * rad_dist_u + 2.0 * p1_ * mxy_u
  + p2_ * (rho_u + 2.0 * mx_u);
  (*pointDistorted)[1] = u1 + u1 * rad_dist_u + 2.0 * p2_ * mxy_u
  + p1_ * (rho_u + 2.0 * my_u);
  return true;
}

bool RadialTangentialDistortion::distort(
    const Eigen::Vector2d & pointUndistorted, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian,
    Eigen::Matrix2Xd * parameterJacobian) const {
  // first compute the distorted point
  const float u0 = pointUndistorted[0];
  const float u1 = pointUndistorted[1];
  const float mx_u = u0 * u0;
  const float my_u = u1 * u1;
  const float mxy_u = u0 * u1;
  const float rho_u = mx_u + my_u;
  const float rad_dist_u = k1_ * rho_u + k2_ * rho_u * rho_u;
  (*pointDistorted)[0] = u0 + u0 * rad_dist_u + 2.0 * p1_ * mxy_u
      + p2_ * (rho_u + 2.0 * mx_u);
  (*pointDistorted)[1] = u1 + u1 * rad_dist_u + 2.0 * p2_ * mxy_u
      + p1_ * (rho_u + 2.0 * my_u);

  // next the Jacobian gyr.r.t. changes on the undistorted point
  Eigen::Matrix2d & J = *pointJacobian;
  J(0, 0) = 1 + rad_dist_u + k1_ * 2.0 * mx_u + k2_ * rho_u * 4 * mx_u
      + 2.0 * p1_ * u1 + 6 * p2_ * u0;
  J(1, 0) = k1_ * 2.0 * u0 * u1 + k2_ * 4 * rho_u * u0 * u1 + p1_ * 2.0 * u0
      + 2.0 * p2_ * u1;
  J(0, 1) = J(1, 0);
  J(1, 1) = 1 + rad_dist_u + k1_ * 2.0 * my_u + k2_ * rho_u * 4 * my_u
      + 6 * p1_ * u1 + 2.0 * p2_ * u0;

  if (parameterJacobian) {
    // the Jacobian gyr.r.t. intrinsics parameters
    Eigen::Matrix2Xd & J2 = *parameterJacobian;
    J2.resize(2, NumDistortionIntrinsics);
    const float r2 = rho_u;
    const float r4 = r2 * r2;

    //[ u0*(u0^2 + u1^2), u0*(u0^2 + u1^2)^2,       2*u0*u1, 3*u0^2 + u1^2]
    //[ u1*(u0^2 + u1^2), u1*(u0^2 + u1^2)^2, u0^2 + 3*u1^2,       2*u0*u1]

    J2(0, 0) = u0 * r2;
    J2(0, 1) = u0 * r4;
    J2(0, 2) = 2.0 * u0 * u1;
    J2(0, 3) = r2 + 2.0 * u0 * u0;

    J2(1, 0) = u1 * r2;
    J2(1, 1) = u1 * r4;
    J2(1, 2) = r2 + 2.0 * u1 * u1;
    J2(1, 3) = 2.0 * u0 * u1;
  }
  return true;
}
bool RadialTangentialDistortion::distort(const Eigen::Vector2f & pointUndistorted,
                                         Eigen::Vector2f * pointDistorted,
                                         Eigen::Matrix2f * pointJacobian,
                                         Eigen::Matrix2Xf * parameterJacobian) const {
    ////针对k1,k2,k3,p1,p2而言 r^2 = x^2 + y^2
    ////残差2*1 归一化坐标的2维： min F(x,y) = 0.5*r(x,y)^2 约束： x^2+y^2 <=
    ///      r.x = x*(1 + k1*r^2 + k2*r^4 ) + 2*p1*x*y + p2*(r^2 + 2*x^2) - x0
    ////     r.y = y*(1 + k1*r^2 + k2*r^4 ) + 2*p2*x*y + p1*(r^2 + 2*y^2) - y0
    ////迭代法求,这里用的狗腿。下面是雅克比矩阵J,代码里是先给J矩阵径向部分然后再给切向部分
    ////J00 = div(r.x)/div(x) = (1 + k1*r^2 + k2*r^4 ) + (k1 + 2*k2*r^2 )*2*x^2 + 2*p1*y + 6*p2*x
    ////J01 = div(r.x)/div(y) = (k1 + 2*k2*r^2)*x*y + 2*p1*x + 2*p2*y
    ////J10 = div(r.y)/div(x) = (k1 + 2*k2*r^2)*x*y + 2*p1*x + 2*p2*y
    ////J11 = div(r.y)/div(y) = (1 + k1*r^2 + k2*r^4) + (k1 + 2*k2*r^2)*2*y^2 + 6*p1*y + 2*p2*x
  // first compute the distorted point
  const float u0 = pointUndistorted[0];
  const float u1 = pointUndistorted[1];
  const float mx_u = u0 * u0;//x^2
  const float my_u = u1 * u1;//y^2
  const float mxy_u = u0 * u1;//xy
  const float rho_u = mx_u + my_u;//r^2
  const float rad_dist_u = k1_ * rho_u + k2_ * rho_u * rho_u;//k1*r^2 + k2*r^4
  (*pointDistorted)[0] = u0 + u0 * rad_dist_u + 2.0 * p1_ * mxy_u
  + p2_ * (rho_u + 2.0 * mx_u);
  (*pointDistorted)[1] = u1 + u1 * rad_dist_u + 2.0 * p2_ * mxy_u
  + p1_ * (rho_u + 2.0 * my_u);

  // next the Jacobian gyr.r.t. changes on the undistorted point
  Eigen::Matrix2f & J = *pointJacobian;
  J(0, 0) = 1 + rad_dist_u + k1_ * 2.0 * mx_u + k2_ * rho_u * 4 * mx_u
  + 2.0 * p1_ * u1 + 6 * p2_ * u0;
  J(1, 0) = k1_ * 2.0 * u0 * u1 + k2_ * 4 * rho_u * u0 * u1 + p1_ * 2.0 * u0
  + 2.0 * p2_ * u1;
  J(0, 1) = J(1, 0);
  J(1, 1) = 1 + rad_dist_u + k1_ * 2.0 * my_u + k2_ * rho_u * 4 * my_u
  + 6 * p1_ * u1 + 2.0 * p2_ * u0;

  if (parameterJacobian) {
    // the Jacobian gyr.r.t. intrinsics parameters
    Eigen::Matrix2Xf & J2 = *parameterJacobian;
    J2.resize(2, NumDistortionIntrinsics);
    const float r2 = rho_u;
    const float r4 = r2 * r2;

    //[ u0*(u0^2 + u1^2), u0*(u0^2 + u1^2)^2,       2*u0*u1, 3*u0^2 + u1^2]
    //[ u1*(u0^2 + u1^2), u1*(u0^2 + u1^2)^2, u0^2 + 3*u1^2,       2*u0*u1]

    J2(0, 0) = u0 * r2;
    J2(0, 1) = u0 * r4;
    J2(0, 2) = 2.0 * u0 * u1;
    J2(0, 3) = r2 + 2.0 * u0 * u0;

    J2(1, 0) = u1 * r2;
    J2(1, 1) = u1 * r4;
    J2(1, 2) = r2 + 2.0 * u1 * u1;
    J2(1, 3) = 2.0 * u0 * u1;
  }
  return true;
}

bool RadialTangentialDistortion::distortWithExternalParameters(
    const Eigen::Vector2d & pointUndistorted,
    const Eigen::VectorXd & parameters, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian,
    Eigen::Matrix2Xd * parameterJacobian) const {
  const float k1 = parameters[0];
  const float k2 = parameters[1];
  const float p1 = parameters[2];
  const float p2 = parameters[3];
  // first compute the distorted point
  const float u0 = pointUndistorted[0];
  const float u1 = pointUndistorted[1];
  const float mx_u = u0 * u0;
  const float my_u = u1 * u1;
  const float mxy_u = u0 * u1;
  const float rho_u = mx_u + my_u;
  const float rad_dist_u = k1 * rho_u + k2 * rho_u * rho_u;
  (*pointDistorted)[0] = u0 + u0 * rad_dist_u + 2.0 * p1 * mxy_u
      + p2 * (rho_u + 2.0 * mx_u);
  (*pointDistorted)[1] = u1 + u1 * rad_dist_u + 2.0 * p2 * mxy_u
      + p1 * (rho_u + 2.0 * my_u);

  // next the Jacobian gyr.r.t. changes on the undistorted point
  Eigen::Matrix2d & J = *pointJacobian;
  J(0, 0) = 1 + rad_dist_u + k1 * 2.0 * mx_u + k2 * rho_u * 4 * mx_u
      + 2.0 * p1 * u1 + 6 * p2 * u0;
  J(1, 0) = k1 * 2.0 * u0 * u1 + k2 * 4 * rho_u * u0 * u1 + p1 * 2.0 * u0
      + 2.0 * p2 * u1;
  J(0, 1) = J(1, 0);
  J(1, 1) = 1 + rad_dist_u + k1 * 2.0 * my_u + k2 * rho_u * 4 * my_u
      + 6 * p1 * u1 + 2.0 * p2 * u0;

  if (parameterJacobian) {
    // the Jacobian gyr.r.t. intrinsics parameters
    Eigen::Matrix2Xd & J2 = *parameterJacobian;
    J2.resize(2, NumDistortionIntrinsics);
    const float r2 = rho_u;
    const float r4 = r2 * r2;

    //[ u0*(u0^2 + u1^2), u0*(u0^2 + u1^2)^2,       2*u0*u1, 3*u0^2 + u1^2]
    //[ u1*(u0^2 + u1^2), u1*(u0^2 + u1^2)^2, u0^2 + 3*u1^2,       2*u0*u1]

    J2(0, 0) = u0 * r2;
    J2(0, 1) = u0 * r4;
    J2(0, 2) = 2.0 * u0 * u1;
    J2(0, 3) = r2 + 2.0 * u0 * u0;

    J2(1, 0) = u1 * r2;
    J2(1, 1) = u1 * r4;
    J2(1, 2) = r2 + 2.0 * u1 * u1;
    J2(1, 3) = 2.0 * u0 * u1;
  }
  return true;
}
bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2d & pointDistorted,
    Eigen::Vector2d * pointUndistorted) const {
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted;  // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {
    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Matrix2d E2 = (E.transpose() * E);
    Eigen::Vector2d du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }
  }
  *pointUndistorted = x_bar;

  if (!success) {
    // std::cout<<(E.transpose() * E)<<std::endl;
  }
  return success;
}

bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2f & pointDistorted,
    Eigen::Vector2f * pointUndistorted) const {
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2f x_bar = pointDistorted;  // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2f E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {
    Eigen::Vector2f x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2f e(pointDistorted - x_tmp);
    Eigen::Matrix2f E2 = (E.transpose() * E);
    Eigen::Vector2f du = E2.inverse() * E.transpose() * e;

    x_bar += du;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }
  }
  *pointUndistorted = x_bar;

  if (!success) {
    // std::cout<<(E.transpose() * E)<<std::endl;
  }
  return success;
}

bool RadialTangentialDistortion::undistort(
    const Eigen::Vector2d & pointDistorted, Eigen::Vector2d * pointUndistorted,
    Eigen::Matrix2d * pointJacobian) const {
  // this is expensive: we solve with Gauss-Newton...
  Eigen::Vector2d x_bar = pointDistorted;  // initialise at distorted point
  const int n = 5;  // just 5 iterations max.
  Eigen::Matrix2d E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; i++) {
    Eigen::Vector2d x_tmp;

    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2d e(pointDistorted - x_tmp);
    Eigen::Vector2d dx = (E.transpose() * E).inverse() * E.transpose() * e;

    x_bar += dx;

    const double chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-15) {
      success = true;
      break;
    }
  }
  *pointUndistorted = x_bar;

  // the Jacobian of the inverse map is simply the inverse Jacobian.
  *pointJacobian = E.inverse();

  return success;
}

bool RadialTangentialDistortion::undistort(const Eigen::Vector2f& pointDistorted,
                                           Eigen::Vector2f* pointUndistorted,
                                           Eigen::Matrix2f* pointJacobian) const {
  Eigen::Vector2f x_bar = pointDistorted;  // initialize at distorted point
  const int n = 5;  // Max 5 iterations
  Eigen::Matrix2f E;  // error Jacobian

  bool success = false;
  for (int i = 0; i < n; ++i) {
    Eigen::Vector2f x_tmp;
    distort(x_bar, &x_tmp, &E);

    Eigen::Vector2f e(pointDistorted - x_tmp);
    Eigen::Vector2f dx = (E.transpose() * E).inverse() * E.transpose() * e;

    x_bar += dx;

    const float chi2 = e.dot(e);
    if (chi2 < 1e-4) {
      success = true;
    }
    if (chi2 < 1e-8) {
      success = true;
      break;
    }
  }
  *pointUndistorted = x_bar;

  // the Jacobian of the inverse map is simply the inverse Jacobian.
  *pointJacobian = E.inverse();

  return success;
}

}  // namespace cameras
}  // namespace vio
