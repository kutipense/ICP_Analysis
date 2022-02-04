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
    T inSrcPoint[3], outSrcPoint[3];
    T inDstPoint[3], outDstPoint[3];

    fillVector(m_sourcePoint, inSrcPoint);
    fillVector(m_targetPoint, inDstPoint);

    getPoseIncrement(const_cast<T*>(pose), inSrcPoint, outSrcPoint);

    T invRot[3] = {-pose[0], -pose[1], -pose[2]};
    ceres::AngleAxisRotatePoint(invRot, inDstPoint, outDstPoint);

    residuals[0] = (T(m_sourceNormal[0] + m_targetNormal[0]) * (outSrcPoint[0] - outDstPoint[0]) +
                    T(m_sourceNormal[1] + m_targetNormal[1]) * (outSrcPoint[1] - outDstPoint[1]) +
                    T(m_sourceNormal[2] + m_targetNormal[2]) * (outSrcPoint[2] - outDstPoint[2])) *
                   T(m_weight);

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