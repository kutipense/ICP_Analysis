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

#endif