#include <optimizing/linear/SymmetricLinear.h>

#include <iostream>

namespace linear {

using namespace Eigen;

SymmetricLinear::SymmetricLinear(const VVector3fConstRef sPoints, const VVector3fConstRef tPoints,
                                 const VVector3fConstRef sNormals, const VVector3fConstRef tNormals)
    : sPoints_(sPoints), tPoints_(tPoints), sNormals_(sNormals), tNormals_(tNormals) {}

Matrix4f SymmetricLinear::operator()() {
  using Matrix4x6 = Eigen::Matrix<float, 4, 6>;

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
    // clang-format off
    A.block<1, 3>(0, 0) = (s + t).cross(n);
    A.block<1, 3>(0, 3) = n;
    A.block(1, 0, 3, 6) << 0.0f, s.z(), -s.y(), 1.0f, 0.0f, 0.0f, // point to point constraints
                           -s.z(), 0.0f, s.x(), 0.0f, 1.0f, 0.0f,
                           s.y(), -s.x(), 0.0f, 0.0f, 0.0f, 1.0f;
    // clang-format on
    M.rankUpdate(A.transpose());

    Vector4f b;
    b << (t - s).dot(n), t - s;
    ATb += A.transpose() * b;

    // std::cout << "it: " << i << std::endl;
    // std::cout << A.transpose() * A << std::endl;
    // std::cout << ATA << std::endl;
    // std::cout << std::endl;
  }

  JacobiSVD<MatrixXf> svd(ATA, ComputeFullV | ComputeFullU);
  Eigen::VectorXf     x = svd.solve(ATb);

  // const Eigen::Vector6f x = M.ldlt().solve(ATb);

  // // Build the system
  // MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
  // VectorXf b = VectorXf::Zero(4 * nPoints);

  // for (unsigned i = 0; i < nPoints; i++) {
  //   const auto& s = sPoints_[i];
  //   const auto& t = tPoints_[i];

  //   Eigen::Vector3f n = sNormals_[i] + tNormals_[i];

  //   // Add the point-to-plane constraints to the system
  //   A.block<1, 3>(4 * i, 0) = (s + t).cross(n);
  //   A.block<1, 3>(4 * i, 3) = n;
  //   b(4 * i)                = (t - s).dot(n);

  //   // Add the point-to-point constraints to the system
  //   A.block<3, 3>(4 * i + 1, 0) << 0.0f, s.z(), -s.y(), -s.z(), 0.0f, s.x(), s.y(), -s.x(), 0.0f;
  //   A.block<3, 3>(4 * i + 1, 3).setIdentity();
  //   b.segment<3>(4 * i + 1) = t - s;
  //   // // Optionally, apply a higher weight to point-to-plane correspondences
  //   // A.block<1, 6>(4 * i, 0) *= this->weight;
  //   // b(4 * i) *= this->weight;
  // }

  // // Solve the system
  // MatrixXf            ATA = A.transpose() * A;
  // VectorXf            ATb = A.transpose() * b;
  // JacobiSVD<MatrixXf> svd(ATA, ComputeFullV | ComputeFullU);
  // Eigen::VectorXf     x = svd.solve(ATb);

  // Build the pose matrix
  const Eigen::AngleAxisf rot_x = AngleAxisf(x(0), Eigen::Vector3f::UnitX()),
                          rot_y = AngleAxisf(x(1), Eigen::Vector3f::UnitY()),
                          rot_z = AngleAxisf(x(2), Eigen::Vector3f::UnitZ());

  const float             theta = std::atan(x.norm());
  const Eigen::AngleAxisf rot(theta, x.head(3).normalized());

  const Vector3f       tl_vector = x.tail(3);  // * std::cos(theta);
  Eigen::Translation3f tl(tl_vector);
  Eigen::Affine3f      tr = rot_z * rot_y * rot_x * tl * rot_z * rot_y * rot_x;

  return tr.matrix();
}

}  // namespace linear