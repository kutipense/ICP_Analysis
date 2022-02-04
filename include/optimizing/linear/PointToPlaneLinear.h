#if !defined(__POINT_TO_PLANE_LINEAR_H__)
#define __POINT_TO_PLANE_LINEAR_H__

#include <external/Eigen.h>

namespace linear {

struct PointToPlaneLinear {
  typedef const std::vector<Eigen::Vector3f>& VVector3fConstRef;

  PointToPlaneLinear(const VVector3fConstRef sPoints, const VVector3fConstRef tPoints,
                     const VVector3fConstRef tNormals);

  Eigen::Matrix4f operator()();

 private:
  VVector3fConstRef sPoints_, tPoints_, tNormals_;
};
}  // namespace linear

#endif  // __POINT_TO_PLANE_LINEAR_H__
