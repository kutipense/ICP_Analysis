#ifndef __EIGEN_EXTERNAL_H__
#define __EIGEN_EXTERNAL_H__

#ifndef VERBOSE
//#define VERBOSE(msg) {std::cout << msg << std::endl;}
#define VERBOSE(msg)
#endif

#ifndef ASSERT
#define ASSERT(a)                                                                                           \
  {                                                                                                         \
    if (!a) {                                                                                               \
      std::cerr << "Error:\nFile: " << __FILE__ << "\nLine: " << __LINE__ << "\nFunction: " << __FUNCTION__ \
                << std::endl;                                                                               \
      while (1)                                                                                             \
        ;                                                                                                   \
    }                                                                                                       \
  }
#endif

#ifndef SAFE_DELETE
#define SAFE_DELETE(ptr)  \
  {                       \
    if (ptr != nullptr) { \
      delete ptr;         \
      ptr = nullptr;      \
    }                     \
  }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(ptr) \
  {                            \
    if (ptr != nullptr) {      \
      delete[] ptr;            \
      ptr = nullptr;           \
    }                          \
  }
#endif

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif

#ifndef M_PI
#define M_PI 3.14159265359
#endif

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/StdVector>
#include <unsupported/Eigen/NonLinearOptimization>

typedef Eigen::Matrix<unsigned char, 4, 1> Vector4uc;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Vector4uc)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::MatrixXf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Quaternionf)

namespace Eigen {
typedef Eigen::Matrix<float, 6, 1>  Vector6f;
typedef Eigen::Matrix<float, 6, 6>  Matrix6f;
typedef Eigen::Matrix<float, 4, 6>  Matrix4x6;
typedef Eigen::Matrix<float, 3, 6>  Matrix3x6;
typedef Eigen::Matrix<u_char, 4, 1> Vector4uc;

inline std::vector<Eigen::Vector3f> transformPoints(const std::vector<Eigen::Vector3f> &sourcePoints,
                                                    const Eigen::Matrix4f &             pose) {
  std::vector<Eigen::Vector3f> transformedPoints;
  transformedPoints.reserve(sourcePoints.size());

  const auto rotation    = pose.block(0, 0, 3, 3);
  const auto translation = pose.block(0, 3, 3, 1);

  for (const auto &point : sourcePoints) {
    Eigen::Vector3f _point(point[0], point[1], point[2]);
    auto const      v = rotation * _point + translation;
    transformedPoints.emplace_back(v);
  }

  return transformedPoints;
}

inline std::vector<Eigen::Vector3f> transformNormals(const std::vector<Eigen::Vector3f> &sourceNormals,
                                                     const Eigen::Matrix4f &             pose) {
  std::vector<Eigen::Vector3f> transformedNormals;
  transformedNormals.reserve(sourceNormals.size());

  const auto rotation = pose.block(0, 0, 3, 3);
  for (const auto &normal : sourceNormals) {
    Eigen::Vector3f _normal(normal[0], normal[1], normal[2]);
    auto const      v = rotation.inverse().transpose() * _normal;
    transformedNormals.emplace_back(v);
  }

  return transformedNormals;
}
}  // namespace Eigen

using namespace Eigen;

template <typename T, unsigned int n, unsigned m>
std::istream &operator>>(std::istream &in, Matrix<T, n, m> &other) {
  for (unsigned int i = 0; i < other.rows(); i++)
    for (unsigned int j = 0; j < other.cols(); j++) in >> other(i, j);
  return in;
}

template <typename T, unsigned int n, unsigned m>
std::ostream &operator<<(std::ostream &out, const Matrix<T, n, m> &other) {
  std::fixed(out);
  for (int i = 0; i < other.rows(); i++) {
    out << other(i, 0);
    for (int j = 1; j < other.cols(); j++) { out << "\t" << other(i, j); }
    out << std::endl;
  }
  return out;
}

template <typename T>
std::istream &operator>>(std::istream &in, Eigen::Quaternion<T> &other) {
  in >> other.x() >> other.y() >> other.z() >> other.w();
  return in;
}

template <typename T>
std::ostream &operator<<(std::ostream &out, const Eigen::Quaternion<T> &other) {
  std::fixed(out);
  out << other.x() << "\t" << other.y() << "\t" << other.z() << "\t" << other.w();
  return out;
}

#endif