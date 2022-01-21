#ifndef __LINEAR_OPTIMIZER_H__
#define __LINEAR_OPTIMIZER_H__

#include <ceres/rotation.h>

template <typename T, typename M, typename SamplerType, typename DiscardType, typename MatcherType>
class LinearOptimizer : public Optimizer<T, M, SamplerType, DiscardType, MatcherType> {
  using Optimizer::Optimizer;

 public:
  virtual void optimize(Eigen::Matrix4f& initialPose) override {}
};

/**
 * ICP optimizer - using linear least-squares for optimization.
 */
class LinearICPOptimizer : public ICPOptimizer {
 public:
  LinearICPOptimizer() {}

  virtual void estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f& initialPose) override {
    // Build the index of the FLANN tree (for fast nearest neighbor lookup).
    m_nearestNeighborSearch->buildIndex(target.getPoints());

    // The initial estimate can be given as an argument.
    Matrix4f estimatedPose = initialPose;

    for (int i = 0; i < m_nIterations; ++i) {
      // Compute the matches.
      std::cout << "Matching points ..." << std::endl;
      clock_t begin = clock();

      auto transformedPoints  = transformPoints(source.getPoints(), estimatedPose);
      auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

      auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
      pruneCorrespondences(transformedNormals, target.getNormals(), matches);

      clock_t end         = clock();
      double  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
      std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

      std::vector<Vector3f> sourcePoints;
      std::vector<Vector3f> targetPoints;

      // Add all matches to the sourcePoints and targetPoints vector,
      // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
      // the matches vector holds the index of the matching target point.
      for (int j = 0; j < transformedPoints.size(); j++) {
        const auto& match = matches[j];
        if (match.idx >= 0) {
          sourcePoints.push_back(transformedPoints[j]);
          targetPoints.push_back(target.getPoints()[match.idx]);
        }
      }

      // Estimate the new pose
      if (m_bUsePointToPlaneConstraints) {
        estimatedPose = estimatePosePointToPlane(sourcePoints, targetPoints, target.getNormals()) * estimatedPose;
      } else {
        estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
      }

      std::cout << "Optimization iteration done." << std::endl;
    }

    // Store result
    initialPose = estimatedPose;
  }

 private:
  Matrix4f estimatePosePointToPoint(const std::vector<Vector3f>& sourcePoints,
                                    const std::vector<Vector3f>& targetPoints) {
    ProcrustesAligner procrustAligner;
    Matrix4f          estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

    return estimatedPose;
  }

  Matrix4f estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints,
                                    const std::vector<Vector3f>& targetPoints,
                                    const std::vector<Vector3f>& targetNormals) {
    const unsigned nPoints = sourcePoints.size();

    // Build the system
    MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
    VectorXf b = VectorXf::Zero(4 * nPoints);

    for (unsigned i = 0; i < nPoints; i++) {
      const auto& s = sourcePoints[i];
      const auto& d = targetPoints[i];
      const auto& n = targetNormals[i];

      // TODO: Add the point-to-plane constraints to the system
      A.block(i, 0, 1, 6) << n.z() * s.y() - n.y() * s.z(), n.x() * s.z() - n.z() * s.x(),
          n.y() * s.x() - n.x() * s.y(), n.x(), n.y(), n.z();
      b.block(i, 0, 1, 1) << n.x() * d.x() + n.y() * d.y() + n.z() * d.z() - n.x() * s.x() - n.y() * s.y() -
                                 n.z() * s.z();

      // TODO: Add the point-to-point constraints to the system
      A.block(i + 1, 0, 1, 6) << 0, s.z(), s.y(), 1, 0, 0;
      A.block(i + 2, 0, 1, 6) << -s.z(), 0, s.x(), 0, 1, 0;
      A.block(i + 3, 0, 1, 6) << s.y(), -s.x(), 0, 0, 0, 1;
      b.block(i + 1, 0, 1, 1) << -s.x() + d.x();
      b.block(i + 2, 0, 1, 1) << -s.y() + d.y();
      b.block(i + 3, 0, 1, 1) << -s.z() + d.z();
      // TODO: Optionally, apply a higher weight to point-to-plane correspondences
    }

    // TODO: Solve the system
    Eigen::VectorXf x(6);

    JacobiSVD<MatrixXf> svd(A, ComputeFullV | ComputeFullU);
    x = svd.solve(b);

    float alpha = x(0), beta = x(1), gamma = x(2);

    // Build the pose matrix
    Matrix3f rotation = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
                        AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
                        AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();

    Vector3f translation = x.tail(3);

    // TODO: Build the pose matrix using the rotation and translation matrices
    Matrix4f estimatedPose          = Matrix4f::Identity();
    estimatedPose.block(0, 0, 3, 3) = rotation;
    estimatedPose.block(0, 3, 3, 1) = translation;

    return estimatedPose;
  }
};

#endif