#include <optimizing/linear/Symmetric.h>

#include <iostream>

namespace linear {

using namespace Eigen;

Symmetric::Symmetric(const VVector3fConstRef sPoints, const VVector3fConstRef tPoints, const VVector3fConstRef sNormals,
                     const VVector3fConstRef tNormals)
    : sPoints_(sPoints), tPoints_(tPoints), sNormals_(sNormals), tNormals_(tNormals) {}

Matrix4f Symmetric::operator()() {
  const size_t nPoints = sPoints_.size();

  // Build the system
  Matrix6f ATA = Matrix6f::Zero();
  Vector6f ATb = Vector6f::Zero();

  auto M = ATA.selfadjointView<Eigen::Upper>();

  for (size_t i = 0; i < nPoints; i++) {
    const auto& s = sPoints_[i];
    const auto& t = tPoints_[i];

    Eigen::Vector3f n = tNormals_[i] + sNormals_[i];

    if (!s.array().isFinite().all() || !t.array().isFinite().all() || !n.array().isFinite().all()) continue;

    Matrix4x6 A;
    Vector4f  b;

    // clang-format off
    A.block<1, 3>(0, 0) = (s + t).cross(n);
    A.block<1, 3>(0, 3) = n;
    A.block(1, 0, 3, 6) << 0.0f, s.z(), -s.y(), 1.0f, 0.0f, 0.0f, // point to point constraints
                           -s.z(), 0.0f, s.x(), 0.0f, 1.0f, 0.0f,
                           s.y(), -s.x(), 0.0f, 0.0f, 0.0f, 1.0f;
    // clang-format on

    M.rankUpdate(A.transpose());

    b << (t - s).dot(n), t - s;
    ATb += A.transpose() * b;
  }

  const Eigen::Vector6f x = M.ldlt().solve(ATb);

  const float             theta = std::atan(x.norm());
  const Eigen::AngleAxisf rot(theta, x.head(3).normalized());
  Eigen::Translation3f    tl(x.tail(3) * std::cos(theta));

  Eigen::Affine3f tr = rot * tl * rot;
  return tr.matrix();
}

}  // namespace linear