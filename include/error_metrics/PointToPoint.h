#ifndef __POINT_TO_POINT_H__
#define __POINT_TO_POINT_H__

#include <ceres/ceres.h>
#include <error_metrics/Helpers.h>

#include <Eigen/Dense>

class PointToPointConstraint {
 public:
  PointToPointConstraint(const Eigen::Vector3f& sourcePoint, const Eigen::Vector3f& targetPoint, const float weight)
      : m_sourcePoint{sourcePoint}, m_targetPoint{targetPoint}, m_weight{weight} {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    T input[3], output[3];
    fillVector(m_sourcePoint, input);

    getPoseIncrement(const_cast<T*>(pose), input, output);

    residuals[0] = (output[0] - T(m_targetPoint(0))) * T(m_weight);
    residuals[1] = (output[1] - T(m_targetPoint(1))) * T(m_weight);
    residuals[2] = (output[2] - T(m_targetPoint(2))) * T(m_weight);

    return true;
  }

  static ceres::CostFunction* create(const Eigen::Vector3f& sourcePoint, const Eigen::Vector3f& targetPoint,
                                     const float weight) {
    return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
        new PointToPointConstraint(sourcePoint, targetPoint, weight));
  }

 protected:
  const Eigen::Vector3f m_sourcePoint;
  const Eigen::Vector3f m_targetPoint;
  const float           m_weight;
};

#endif