#include <optimizing/linear/PointToPoint.h>

#include <iostream>

namespace linear {

using namespace Eigen;

PointToPoint::PointToPoint(const VVector3fConstRef sPoints, const VVector3fConstRef tPoints)
    : sPoints_(sPoints), tPoints_(tPoints) {}

Matrix4f PointToPoint::operator()() {
  Matrix6f ATA = Matrix6f::Zero();
  Vector6f ATb = Vector6f::Zero();

  auto M = ATA.selfadjointView<Eigen::Upper>();

  for (size_t i = 0; i < sPoints_.size(); i++) {
    const auto& s = sPoints_[i];
    const auto& t = tPoints_[i];

    if (!s.array().isFinite().all() || !t.array().isFinite().all()) continue;

    Matrix3x6 A;
    Vector3f  b;

    // clang-format off
    A << 0.0f, s.z(), -s.y(), 1.0f, 0.0f, 0.0f, // point to point constraints
         -s.z(), 0.0f, s.x(), 0.0f, 1.0f, 0.0f,
         s.y(), -s.x(), 0.0f, 0.0f, 0.0f, 1.0f;
    // clang-format on

    M.rankUpdate(A.transpose());

    b << t - s;
    ATb += A.transpose() * b;
  }

  const Eigen::Vector6f x = M.ldlt().solve(ATb);

  // Build the pose matrix
  Matrix3f rotation = AngleAxisf(x(0), Eigen::Vector3f::UnitX()).toRotationMatrix() *
                      AngleAxisf(x(1), Eigen::Vector3f::UnitY()).toRotationMatrix() *
                      AngleAxisf(x(2), Eigen::Vector3f::UnitZ()).toRotationMatrix();

  Eigen::Vector3f translation = x.tail(3);

  // Build the pose matrix using the rotation and translation matrices
  Matrix4f estimatedPose          = Matrix4f::Identity();
  estimatedPose.block(0, 0, 3, 3) = rotation;
  estimatedPose.block(0, 3, 3, 1) = translation;

  return estimatedPose;
}

}  // namespace linear