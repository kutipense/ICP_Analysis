#ifndef __ERROR_HELPERS_H__
#define __ERROR_HELPERS_H__

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>

template <typename T>
inline void fillVector(const Eigen::Vector3f& input, T* output) {
  output[0] = T(input[0]);
  output[1] = T(input[1]);
  output[2] = T(input[2]);
}

template <typename T>
inline void getPoseIncrement(T* pose, T* inputPoint, T* outputPoint) {
  const T* rotation    = pose;
  const T* translation = pose + 3;

  T temp[3];
  ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

  outputPoint[0] = temp[0] + translation[0];
  outputPoint[1] = temp[1] + translation[1];
  outputPoint[2] = temp[2] + translation[2];
}

// template <typename T>
// inline void transformPoint(T* pose, T* outputPoint) {
//   const Eigen::Vector6f   x(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
//   const float             theta = std::atan(x.head(3).norm());
//   const Eigen::AngleAxisf rot(theta, x.head(3).normalized());
//   Eigen::Translation3f    tl(x.tail(3) * std::cos(theta));

//   Eigen::Affine3f tr  = rot * tl * rot;
//   Eigen::Vector3f out = tr * x.tail(3);

//   outputPoint[0] = out[0];
//   outputPoint[1] = out[1];
//   outputPoint[2] = out[2];
// }

#endif