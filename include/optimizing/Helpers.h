#ifndef __OPTIMIZER_HELPERS_H__
#define __OPTIMIZER_HELPERS_H__
#include <ceres/rotation.h>
#include <data_types/VertexList.h>
template <typename T>
class PoseIncrement {
 public:
  explicit PoseIncrement(T* const array) : m_array{array} {}

  void setZero() {
    for (int i = 0; i < 6; ++i) m_array[i] = T(0);
  }

  T* getData() const { return m_array; }

  /**
   * Applies the pose increment onto the input point and produces transformed output point.
   * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
   * beforehand).
   */
  void apply(T* inputPoint, T* outputPoint) const {
    // pose[0,1,2] is angle-axis rotation.
    // pose[3,4,5] is translation.
    const T* rotation    = m_array;
    const T* translation = m_array + 3;

    T temp[3];
    ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

    outputPoint[0] = temp[0] + translation[0];
    outputPoint[1] = temp[1] + translation[1];
    outputPoint[2] = temp[2] + translation[2];
  }

  /**
   * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
   * transformation 4x4 matrix.
   */
  static Matrix4f convertToMatrix(const PoseIncrement<double>& poseIncrement) {
    // pose[0,1,2] is angle-axis rotation.
    // pose[3,4,5] is translation.
    double* pose        = poseIncrement.getData();
    double* rotation    = pose;
    double* translation = pose + 3;

    // Convert the rotation from SO3 to matrix notation (with column-major storage).
    double rotationMatrix[9];
    ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

    // Create the 4x4 transformation matrix.
    Matrix4f matrix;
    matrix.setIdentity();
    matrix(0, 0) = float(rotationMatrix[0]);
    matrix(0, 1) = float(rotationMatrix[3]);
    matrix(0, 2) = float(rotationMatrix[6]);
    matrix(0, 3) = float(translation[0]);
    matrix(1, 0) = float(rotationMatrix[1]);
    matrix(1, 1) = float(rotationMatrix[4]);
    matrix(1, 2) = float(rotationMatrix[7]);
    matrix(1, 3) = float(translation[1]);
    matrix(2, 0) = float(rotationMatrix[2]);
    matrix(2, 1) = float(rotationMatrix[5]);
    matrix(2, 2) = float(rotationMatrix[8]);
    matrix(2, 3) = float(translation[2]);

    return matrix;
  }

 private:
  T* m_array;
};

class ProcrustesAligner {
 public:
  Matrix4f estimatePose(const VectorEigen3f& sourcePoints, const VectorEigen3f& targetPoints) {
    ASSERT(sourcePoints.size() == targetPoints.size() &&
           "The number of source and target points should be the same, since every source point is matched with "
           "corresponding target point.");

    // We estimate the pose between source and target points using Procrustes algorithm.
    // Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
    // from source points to target points.

    auto sourceMean = computeMean(sourcePoints);
    auto targetMean = computeMean(targetPoints);

    Matrix3f rotation    = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
    Vector3f translation = computeTranslation(sourceMean, targetMean);

    // To apply the pose to point x on shape X in the case of Procrustes, we execute:
    // 1. Translation of a point to the shape Y: x' = x + t
    // 2. Rotation of the point around the mean of shape Y:
    //    y = R (x' - yMean) + yMean = R (x + t - yMean) + yMean = R x + (R t - R yMean + yMean)

    Matrix4f estimatedPose          = Matrix4f::Identity();
    estimatedPose.block(0, 0, 3, 3) = rotation;
    estimatedPose.block(0, 3, 3, 1) = rotation * translation - rotation * targetMean + targetMean;

    return estimatedPose;
  }

 private:
  Vector3f computeMean(const VectorEigen3f& points) {
    // Compute the mean of input points.
    const unsigned nPoints = points.size();
    Vector3f       mean    = Vector3f::Zero();
    for (int i = 0; i < nPoints; ++i) { mean += points[i]; }
    mean /= nPoints;
    return mean;
  }

  Matrix3f estimateRotation(const VectorEigen3f& sourcePoints, const Vector3f& sourceMean,
                            const VectorEigen3f& targetPoints, const Vector3f& targetMean) {
    // Estimate the rotation from source to target points, following the Procrustes algorithm.
    // To compute the singular value decomposition you can use JacobiSVD() from Eigen.
    const unsigned nPoints = sourcePoints.size();
    MatrixXf       sourceMatrix(nPoints, 3);
    MatrixXf       targetMatrix(nPoints, 3);

    for (int i = 0; i < nPoints; ++i) {
      sourceMatrix.block(i, 0, 1, 3) = (sourcePoints[i] - sourceMean).transpose();
      targetMatrix.block(i, 0, 1, 3) = (targetPoints[i] - targetMean).transpose();
    }

    Matrix3f            A = targetMatrix.transpose() * sourceMatrix;
    JacobiSVD<Matrix3f> svd(A, ComputeFullU | ComputeFullV);
    const Matrix3f&     U = svd.matrixU();
    const Matrix3f&     V = svd.matrixV();

    const float d = (U * V.transpose()).determinant();
    Matrix3f    D = Matrix3f::Identity();
    D(2, 2)       = d;

    Matrix3f R = U * D * V.transpose();  // the multiplication by D is necessary since UV' is only orthogonal, but not
                                         // necessarily a rotation matrix
    return R;
  }

  Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean) {
    // Compute the translation vector from source to target points.
    Vector3f translation = targetMean - sourceMean;
    return translation;
  }
};

#endif