#ifndef __OPTIMIZER_HELPERS_H__
#define __OPTIMIZER_HELPERS_H__
#include <ceres/rotation.h>

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

#endif