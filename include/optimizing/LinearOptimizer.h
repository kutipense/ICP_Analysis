#ifndef __LINEAR_OPTIMIZER_H__
#define __LINEAR_OPTIMIZER_H__

#include <ceres/rotation.h>
#include <optimizing/Helpers.h>
#include <optimizing/Optimizer.h>
#include <optimizing/linear/PointToPlaneLinear.h>
#include <optimizing/linear/SymmetricLinear.h>
#include <pcl/common/transforms.h>

template <typename T, typename M, typename SamplerType, typename DiscardType, typename MatcherType>
class LinearOptimizer : public Optimizer<T, M, SamplerType, DiscardType, MatcherType> {
 public:
  LinearOptimizer(typename T::Ptr& source, typename T::Ptr& target,
                  ErrorMetric error_metric = ErrorMetric::PointToPoint, unsigned int m_nIterations = 20,
                  const float weight = 2)
      : Optimizer<T, M, SamplerType, DiscardType, MatcherType>(source, target, error_metric, m_nIterations, weight) {}

  virtual void optimize(Eigen::Matrix4f& initialPose) override {
    typename SamplerType::Ptr sampler  = std::make_shared<SamplerType>(this->source, 0.002f);
    typename T::Ptr           sampled  = sampler->sample();
    typename SamplerType::Ptr sampler2 = std::make_shared<SamplerType>(this->target, 0.002f);
    typename T::Ptr           sampled2 = sampler2->sample();
    typename MatcherType::Ptr matcher;

    auto sourcePC = VertexList::toPCL<pcl::PointXYZ>(this->source->vertices);

    // std::cout << sampled->vertices.size() << std::endl;

    // sampled->exportToOFF("bunnySampledICP.off");
    // sampled2->exportToOFF("bunny45SampledICP.off");

    // The initial estimate can be given as an argument.

    Eigen::AngleAxisd rollAngle(0.5, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    Eigen::Matrix4f fakeTr         = Eigen::Matrix4f::Identity();
    fakeTr.block(0, 0, 3, 3)       = rotationMatrix.cast<float>();
    fakeTr.block(0, 3, 3, 1)       = Eigen::Vector3f(0.0, .0, -0.0);

    auto _t1           = VertexList::toEigen(sampled->vertices);
    auto _t2           = VertexList::toEigen(sampled->normals);
    auto t1            = VertexList::fromEigen(this->transformPoints(_t1, fakeTr));
    auto t2            = VertexList::fromEigen(this->transformNormals(_t2, fakeTr));
    sampled2->vertices = t1->vertices;
    sampled2->normals  = t2->vertices;
    sampled2->exportToOFF("bunnyFake.off");

    Eigen::Matrix4f estimatedPose = initialPose;

    for (size_t i = 0; i < this->m_nIterations; ++i) {
      // Compute the matches.
      // std::cout << "Matching points ..." << std::endl;
      clock_t begin = clock();

      auto transformedPoints  = this->transformPoints(sampled->vertices, estimatedPose);
      auto transformedNormals = this->transformNormals(sampled->normals, estimatedPose);
      auto _transformedPoints = VertexList::fromEigen(transformedPoints);
      matcher                 = std::make_shared<MatcherType>(_transformedPoints, sampled2);
      auto matches            = matcher->match();
      // pruneCorrespondences(transformedNormals, target.getNormals(), matches);

      clock_t end         = clock();
      double  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
      // std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

      std::vector<Vector3f> sourcePoints;
      std::vector<Vector3f> sourceNormals;
      std::vector<Vector3f> targetPoints;
      std::vector<Vector3f> targetNormals;

      auto targetPts  = VertexList::toEigen(sampled2->vertices);
      auto targetNmls = VertexList::toEigen(sampled2->normals);

      if (this->error_metric == ErrorMetric::Symmetric) {
        Eigen::Matrix4f invRotMatrix = Eigen::Matrix4f::Identity();
        invRotMatrix.block<3, 3>(0, 0) << estimatedPose.block<3, 3>(0, 0).transpose();
        // targetPts = this->transformPoints(targetPts, invRotMatrix);
        // targetNmls = this->transformNormals(targetNmls, invRotMatrix);
      }

      // Add all matches to the sourcePoints and targetPoints vector,
      // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
      // the matches vector holds the index of the matching target point.
      for (size_t j = 0; j < transformedPoints.size(); j++) {
        const auto& match = matches->matches[j];
        if (match.idx >= 0) {
          sourcePoints.push_back(transformedPoints[j]);
          sourceNormals.push_back(transformedNormals[j]);

          targetPoints.push_back(targetPts[match.idx]);
          targetNormals.push_back(targetNmls[match.idx]);
        }
      }

      // Estimate the new pose
      if (this->error_metric == ErrorMetric::PointToPlane) {
        linear::PointToPlaneLinear p2plane(sourcePoints, targetPoints, targetNormals);
        estimatedPose = p2plane() * estimatedPose;
      } else if (this->error_metric == ErrorMetric::PointToPoint) {
        estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
      } else if (this->error_metric == ErrorMetric::Symmetric) {
        linear::SymmetricLinear sl(sourcePoints, targetPoints, sourceNormals, targetNormals);
        estimatedPose = sl() * estimatedPose;
      }

      // std::cout << "Optimization iteration done." << std::endl;

      // VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();
      // pcl::transformPointCloud(*sourcePC, *out_cloud, estimatedPose);
      // auto v = VertexList::fromPCL(out_cloud);
      // v->exportToOFF("bunny90ICP_" + std::to_string(i) + ".off");
    }

    // Store result
    initialPose = estimatedPose;
  }

 private:
  Matrix4f estimatePosePointToPoint(const VectorEigen3f& sourcePoints, const VectorEigen3f& targetPoints) {
    ProcrustesAligner procrustAligner;
    Matrix4f          estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

    return estimatedPose;
  }

  Matrix4f estimatePosePointToPlane(const VectorEigen3f& sourcePoints, const VectorEigen3f& targetPoints,
                                    const VectorEigen3f& targetNormals) {
    const unsigned nPoints = sourcePoints.size();

    // Build the system
    MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
    VectorXf b = VectorXf::Zero(4 * nPoints);

    for (unsigned i = 0; i < nPoints; i++) {
      const auto& s = sourcePoints[i];
      const auto& d = targetPoints[i];
      const auto& n = targetNormals[i];

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
      // Optionally, apply a higher weight to point-to-plane correspondences
      A.block<1, 6>(4 * i, 0) *= this->weight;
      b(4 * i) *= this->weight;
    }

    // Solve the system
    MatrixXf            ATA = A.transpose() * A;
    VectorXf            ATb = A.transpose() * b;
    JacobiSVD<MatrixXf> svd(ATA, ComputeFullV | ComputeFullU);
    Eigen::VectorXf     x = svd.solve(ATb);

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

  Matrix4f estimateSymmetric(const VectorEigen3f& sourcePoints, const VectorEigen3f& targetPoints,
                             const VectorEigen3f& sourceNormals, const VectorEigen3f& targetNormals) {
    const unsigned nPoints = sourcePoints.size();

    // Build the system
    MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
    VectorXf b = VectorXf::Zero(4 * nPoints);

    for (unsigned i = 0; i < nPoints; i++) {
      const auto& s  = sourcePoints[i];
      const auto& d  = targetPoints[i];
      const auto& nS = sourceNormals[i];
      const auto& nT = targetNormals[i];

      Eigen::Vector3f n;
      if (nS.dot(nT) >= 0.0)
        n = nS + nT;
      else
        n = nS - nT;

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
      // Optionally, apply a higher weight to point-to-plane correspondences
      A.block<1, 6>(4 * i, 0) *= this->weight;
      b(4 * i) *= this->weight;
    }

    // Solve the system
    MatrixXf            ATA = A.transpose() * A;
    VectorXf            ATb = A.transpose() * b;
    JacobiSVD<MatrixXf> svd(ATA, ComputeFullV | ComputeFullU);
    Eigen::VectorXf     x = svd.solve(ATb);

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
};

#endif