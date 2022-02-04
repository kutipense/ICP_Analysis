#if !defined(__SYMMETRIC_LINEAR_H__)
#define __SYMMETRIC_LINEAR_H__

#include <external/Eigen.h>

namespace linear {
struct Symmetric {
  typedef const std::vector<Eigen::Vector3f>& VVector3fConstRef;

  Symmetric(const VVector3fConstRef sPoints, const VVector3fConstRef tPoints, const VVector3fConstRef sNormals,
            const VVector3fConstRef tNormals);

  Eigen::Matrix4f operator()();

 private:
  VVector3fConstRef sPoints_, tPoints_, sNormals_, tNormals_;
};
}  // namespace linear

#endif  // __SYMMETRIC_LINEAR_H__
