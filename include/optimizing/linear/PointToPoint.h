#if !defined(__POINT_TO_POINT_LINEAR_H__)
#define __POINT_TO_POINT_LINEAR_H__

#include <external/Eigen.h>

namespace linear {
struct PointToPoint {
  typedef const std::vector<Eigen::Vector3f>& VVector3fConstRef;

  PointToPoint(const VVector3fConstRef sPoints, const VVector3fConstRef tPoints);

  Eigen::Matrix4f operator()();

 private:
  VVector3fConstRef sPoints_, tPoints_;
};
}  // namespace linear

#endif  // __POINT_TO_POINT_LINEAR_H__
