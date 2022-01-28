#ifndef __SYMMETRIC_H__
#define __SYMMETRIC_H__

#include <ceres/ceres.h>
#include <error_metrics/Helpers.h>

#include <Eigen/Dense>

class SymmetricConstraint {
 public:
  SymmetricConstraint(const Eigen::Vector3f& sourcePoint, const Eigen::Vector3f& targetPoint,
                      const Eigen::Vector3f& sourceNormal, const Eigen::Vector3f& targetNormal, const float weight)
      : m_sourcePoint{sourcePoint},
        m_targetPoint{targetPoint},
        m_sourceNormal{sourceNormal},
        m_targetNormal{targetNormal},
        m_weight{weight} {}

  template <typename T>
  bool operator()(const T* const pose, T* residuals) const {
    T inputSource[3], inputTarget[3], outputSource[3], outputTarget[3];
    fillVector(m_sourcePoint, inputSource);
    // fillVector(m_targetPoint, inputTarget);

    // T inverseRotation[6] = {-pose[0], -pose[1], -pose[2], T(0), T(0), T(0)};

    getPoseIncrement(const_cast<T*>(pose), inputSource, outputSource);
    // getPoseIncrement(const_cast<T*>(inverseRotation), inputTarget, outputTarget);

    auto xDiff = outputSource[0] - T(m_targetPoint(0));  // outputTarget[0];
    auto yDiff = outputSource[1] - T(m_targetPoint(1));  // outputTarget[1];
    auto zDiff = outputSource[2] - T(m_targetPoint(2));  // outputTarget[2];

    Eigen::Vector3f n;
    if (m_sourceNormal.dot(m_targetNormal) >= 0.0)
      n = m_sourceNormal + m_targetNormal;
    else
      n = m_sourceNormal - m_targetNormal;

    n = m_targetNormal;

    residuals[0] = (T(n[0]) * xDiff + T(n[1]) * yDiff + T(n[2]) * zDiff) * T(m_weight);

    return true;
  }

  static ceres::CostFunction* create(const Eigen::Vector3f& sourcePoint, const Eigen::Vector3f& targetPoint,
                                     const Eigen::Vector3f& sourceNormal, const Eigen::Vector3f& targetNormal,
                                     const float weight) {
    return new ceres::AutoDiffCostFunction<SymmetricConstraint, 1, 6>(
        new SymmetricConstraint(sourcePoint, targetPoint, sourceNormal, targetNormal, weight));
  }

 protected:
  const Eigen::Vector3f m_sourcePoint;
  const Eigen::Vector3f m_targetPoint;
  const Eigen::Vector3f m_sourceNormal;
  const Eigen::Vector3f m_targetNormal;
  const float           m_weight;
};

#endif