#ifndef __LM_OPTIMIZER_H__
#define __LM_OPTIMIZER_H__

#include <ceres/ceres.h>
#include <data_types/VertexList.h>
#include <error_metrics/PointToPlane.h>
#include <error_metrics/PointToPoint.h>
#include <error_metrics/Symmetric.h>
#include <matching/Matcher.h>
#include <optimizing/Helpers.h>
#include <optimizing/Optimizer.h>

#include <Eigen/Dense>

template <typename T, typename M, typename SamplerType, typename DiscardType, typename MatcherType>
class LMOptimizer : public Optimizer<T, M, SamplerType, DiscardType, MatcherType> {
 public:
  LMOptimizer(typename T::Ptr& source, typename T::Ptr& target,
              ErrorMetric error_metric = ErrorMetric::PointToPoint, unsigned int m_nIterations = 20)
      : Optimizer<T, M, SamplerType, DiscardType, MatcherType>(source, target, error_metric, m_nIterations) {}
  virtual void optimize(Eigen::Matrix4f& initialPose) override {
    // Build the index of the FLANN tree (for fast nearest neighbor lookup).
    // m_nearestNeighborSearch->buildIndex(target.getPoints());
    typename SamplerType::Ptr sampler = std::make_shared<SamplerType>(this->source);
    typename T::Ptr           sampled = sampler->sample();
    typename MatcherType::Ptr matcher;
    typename M::Ptr           matched = matcher->match();

    // The initial estimate can be given as an argument.
    Eigen::Matrix4f estimatedPose = initialPose;

    // We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its
    // length presents the rotation angle) and 3 parameters for the translation vector.
    double incrementArray[6];
    auto   poseIncrement = PoseIncrement<double>(incrementArray);
    poseIncrement.setZero();

    for (size_t i = 0; i < this->m_nIterations; ++i) {
      // Compute the matches.
      std::cout << "Matching points ..." << std::endl;
      clock_t begin = clock();

      auto transformedPoints  = this->transformPoints(this->source->vertices, estimatedPose);
      auto _transformedNormals = this->transformNormals(this->source->normals, estimatedPose);
      auto transformedNormals = VertexList::fromEigen(_transformedNormals);
      matcher                 = std::make_shared<MatcherType>(sampled, transformedNormals);
      auto matches            = matcher->match();

      clock_t end         = clock();
      double  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
      std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

      // Prepare point-to-point and point-to-plane constraints.
      ceres::Problem problem;
      auto           vEigen = this->target->toEigen(this->target->vertices);
      auto           nEigen = this->target->toEigen(this->target->normals);
      prepareConstraints(transformedPoints, vEigen, _transformedNormals, nEigen, *matches, poseIncrement, problem);

      // Configure options for the solver.
      ceres::Solver::Options options;
      configureSolver(options);

      // Run the solver (for one iteration).
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << std::endl;
      // std::cout << summary.FullReport() << std::endl;

      // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
      Eigen::Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
      estimatedPose          = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
      poseIncrement.setZero();

      std::cout << "Optimization iteration done." << std::endl;
    }

    // Store result
    initialPose = estimatedPose;
  }

 private:
  void configureSolver(ceres::Solver::Options& options) {
    // Ceres options.
    options.trust_region_strategy_type   = ceres::LEVENBERG_MARQUARDT;
    options.use_nonmonotonic_steps       = false;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = 1;
    options.max_num_iterations           = 1;
    options.num_threads                  = 8;
  }

  void prepareConstraints(const std::vector<Vector3f>& sourcePoints, const VectorEigen3f& targetPoints,
                          const std::vector<Vector3f>& sourceNormals, const VectorEigen3f& targetNormals,
                          const M matches, const PoseIncrement<double>& poseIncrement,
                          ceres::Problem& problem) const {
    const unsigned nPoints = sourcePoints.size();

    for (unsigned i = 0; i < nPoints; ++i) {
      const auto match = matches.matches[i];
      if (match.idx >= 0) {
        const auto& sourcePoint = sourcePoints[i];
        const auto& targetPoint = targetPoints[match.idx];

        if (!sourcePoint.allFinite() || !targetPoint.allFinite()) continue;

        // TODO: Create a new point-to-point cost function and add it as constraint (i.e. residual block)
        // to the Ceres problem.

        if ((sourcePoint - targetPoint).squaredNorm() == 0.0) continue;

        if (this->error_metric == ErrorMetric::PointToPoint)
          problem.AddResidualBlock(PointToPointConstraint::create(sourcePoint, targetPoint, 1), nullptr,
                                   poseIncrement.getData());
        else if (this->error_metric == ErrorMetric::PointToPlane) {
          const auto& targetNormal = targetNormals[match.idx];

          if (!targetNormal.allFinite()) continue;

          // TODO: Create a new point-to-plane cost function and add it as constraint (i.e. residual block)
          // to the Ceres problem.
          // double the weight
          problem.AddResidualBlock(PointToPlaneConstraint::create(sourcePoint, targetPoint, targetNormal, 2), nullptr,
                                   poseIncrement.getData());
        } else if (this->error_metric == ErrorMetric::Symmetric) {
          const auto& targetNormal = targetNormals[match.idx];
          const auto& sourceNormal = sourceNormals[i];
          if (!targetNormal.allFinite() && !sourceNormal.allFinite()) continue;

          // TODO: Create a new point-to-plane cost function and add it as constraint (i.e. residual block)
          // to the Ceres problem.
          // double the weight
          problem.AddResidualBlock(SymmetricConstraint::create(sourcePoint, targetPoint, sourceNormal, targetNormal, 2),
                                   nullptr, poseIncrement.getData());
        }
      }
    }
  }
};

#endif