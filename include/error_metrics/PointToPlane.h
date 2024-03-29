#ifndef __POINT_TO_PLANE_H__
#define __POINT_TO_PLANE_H__

#include <ceres/ceres.h>
#include <error_metrics/Helpers.h>

#include <Eigen/Dense>

class PointToPlaneConstraint {
 public:
  PointToPlaneConstraint(const Eigen::Vector3f& sourcePoint, const Eigen::Vector3f& targetPoint,
                         const Eigen::Vector3f& targetNormal, const float weight)
      : m_sourcePoint{sourcePoint}, m_targetPoint{targetPoint}, m_targetNormal{targetNormal}, m_weight{weight} {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    T input[3], output[3];
    fillVector(m_sourcePoint, input);

    getPoseIncrement(const_cast<T*>(pose), input, output);

    auto xDiff = output[0] - T(m_targetPoint(0));
    auto yDiff = output[1] - T(m_targetPoint(1));
    auto zDiff = output[2] - T(m_targetPoint(2));

    residuals[0] =
        (T(m_targetNormal[0]) * xDiff + T(m_targetNormal[1]) * yDiff + T(m_targetNormal[2]) * zDiff) * T(m_weight);

    return true;
  }

  static ceres::CostFunction* create(const Eigen::Vector3f& sourcePoint, const Eigen::Vector3f& targetPoint,
                                     const Eigen::Vector3f& targetNormal, const float weight) {
    return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
        new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight));
  }

 protected:
  const Eigen::Vector3f m_sourcePoint;
  const Eigen::Vector3f m_targetPoint;
  const Eigen::Vector3f m_targetNormal;
  const float           m_weight;
};

#endif