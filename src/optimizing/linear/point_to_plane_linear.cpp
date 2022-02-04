#include <optimizing/linear/PointToPlaneLinear.h>

#include <iostream>

namespace linear {

using namespace Eigen;

PointToPlaneLinear::PointToPlaneLinear(const VVector3fConstRef sPoints, const VVector3fConstRef tPoints,
                                       const VVector3fConstRef tNormals)
    : sPoints_(sPoints), tPoints_(tPoints), tNormals_(tNormals) {}

Matrix4f PointToPlaneLinear::operator()() {
  const unsigned nPoints = sPoints_.size();

  // Build the system
  MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
  VectorXf b = VectorXf::Zero(4 * nPoints);

  for (unsigned i = 0; i < nPoints; i++) {
    const auto& s = sPoints_[i];
    const auto& d = tPoints_[i];
    const auto& n = tNormals_[i];

    // Add the point-to-plane constraints to the system
    A(4 * i, 0)             = n.z() * s.y() - n.y() * s.z();
    A(4 * i, 1)             = n.x() * s.z() - n.z() * s.x();
    A(4 * i, 2)             = n.y() * s.x() - n.x() * s.y();
    A.block<1, 3>(4 * i, 3) = n;
    b(4 * i)                = (d - s).dot(n);

    // Add the point-to-point constraints to the system
    A.block<3, 3>(4 * i + 1, 0) << 0.0f, s.z(), -s.y(), -s.z(), 0.0f, s.x(), s.y(), -s.x(), 0.0f;
    A.block<3, 3>(4 * i + 1, 3).setIdentity();
    b.segment<3>(4 * i + 1) = d - s;
    // // Optionally, apply a higher weight to point-to-plane correspondences
    // A.block<1, 6>(4 * i, 0) *= this->weight;
    // b(4 * i) *= this->weight;
  }

  // Solve the system
  MatrixXf            ATA = A.transpose() * A;
  VectorXf            ATb = A.transpose() * b;
  JacobiSVD<MatrixXf> svd(ATA, ComputeFullV | ComputeFullU);
  Eigen::VectorXf     x = svd.solve(ATb);
  // std::cout << ATA << std::endl;

  float alpha = x(0), beta = x(1), gamma = x(2);

  // Build the pose matrix
  Matrix3f rotation = AngleAxisf(alpha, Eigen::Vector3f::UnitX()).toRotationMatrix() *
                      AngleAxisf(beta, Eigen::Vector3f::UnitY()).toRotationMatrix() *
                      AngleAxisf(gamma, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  Eigen::Vector3f translation = x.tail(3);

  // Build the pose matrix using the rotation and translation matrices
  Matrix4f estimatedPose          = Matrix4f::Identity();
  estimatedPose.block(0, 0, 3, 3) = rotation;
  estimatedPose.block(0, 3, 3, 1) = translation;

  return estimatedPose;
}

}  // namespace linear