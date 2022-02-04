#ifndef __LM_OPTIMIZER_H__
#define __LM_OPTIMIZER_H__

#include <ceres/ceres.h>
#include <data_types/VertexList.h>
#include <discard/Reject.h>
#include <error_metrics/PointToPlane.h>
#include <error_metrics/PointToPoint.h>
#include <error_metrics/Symmetric.h>
#include <matcher/Matcher.h>
#include <optimizing/Helpers.h>
#include <optimizing/Optimizer.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

template <typename SamplerType, typename DiscardType, typename MatcherType>
class LMOptimizer : public Optimizer<SamplerType, DiscardType, MatcherType> {
 public:
  LMOptimizer(VertexList::Ptr source, VertexList::Ptr target, ErrorMetric error_metric = ErrorMetric::PointToPoint,
              unsigned int m_nIterations = 20, const float weight = 2)
      : Optimizer<SamplerType, DiscardType, MatcherType>(source, target, error_metric, m_nIterations) {}

  virtual void optimize(Eigen::Matrix4f& initialPose) override {
    // Build the index of the FLANN tree (for fast nearest neighbor lookup).
    // m_nearestNeighborSearch->buildIndex(target.getPoints());
    typename SamplerType::Ptr sampler  = std::make_shared<SamplerType>(this->source, 0.002f);
    VertexList::Ptr           sampled  = sampler->sample();
    typename SamplerType::Ptr sampler2 = std::make_shared<SamplerType>(this->target, 0.002f);
    VertexList::Ptr           sampled2 = sampler2->sample();
    typename MatcherType::Ptr matcher;

    std::cout << sampled->vertices.size() << " " << sampled2->vertices.size() << std::endl;

    // The initial estimate can be given as an argument.
    Eigen::Matrix4f estimatedPose = initialPose;

    // We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its
    // length presents the rotation angle) and 3 parameters for the translation vector.
    double incrementArray[6];
    auto   poseIncrement = PoseIncrement<double>(incrementArray);
    poseIncrement.setZero();

    clock_t begin = clock();
    for (size_t i = 0; i < this->m_nIterations; ++i) {
      // if (i % 5 == 0) {
      //   auto                           bunnyPC   = VertexList::toPCL<pcl::PointXYZ>(sampled->vertices);
      //   VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();
      //   pcl::transformPointCloud(*bunnyPC, *out_cloud, estimatedPose);
      //   auto v = VertexList::fromPCL(out_cloud);
      //   v->exportToOFF("bunnyTransformed_" + std::to_string(i) + ".off");
      // }

      auto trSource      = std::make_shared<VertexList>();
      trSource->vertices = this->transformPoints(sampled->vertices, estimatedPose);
      trSource->normals  = this->transformNormals(sampled->normals, estimatedPose);

      matcher      = std::make_shared<MatcherType>(trSource->vertices, sampled2->vertices);
      auto matches = matcher->match();

      discard::Reject::Ptr discarder = std::make_shared<discard::Reject>(trSource, sampled2, matches);
      // discarder->setMaxAngle(0.57);
      // discarder->setMaxDistance(0.01);
      // matches = discarder->discard();

      // Prepare point-to-point and point-to-plane constraints.
      ceres::Problem problem;
      prepareConstraints(trSource->vertices, sampled2->vertices, trSource->normals, sampled2->normals, *matches,
                         poseIncrement, problem);

      // Configure options for the solver.
      ceres::Solver::Options options;
      configureSolver(options);

      // Run the solver (for one iteration).
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      // std::cout << summary.BriefReport() << std::endl;
      // std::cout << summary.FullReport() << std::endl;

      // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
      Eigen::Matrix4d matrix;
      if (this->error_metric == ErrorMetric::Symmetric)
        matrix = PoseIncrement<double>::convertToMatrixSymmetric(poseIncrement);
      else
        matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);

      estimatedPose = matrix.cast<float>() * estimatedPose;
      poseIncrement.setZero();
    }

    clock_t end         = clock();
    double  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;
    // Store result
    initialPose = estimatedPose;
  }

 private:
  void configureSolver(ceres::Solver::Options& options) {
    // Ceres options.
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.use_nonmonotonic_steps     = false;
    // options.linear_solver_type           = ceres::DENSE_QR;
    options.max_num_iterations           = 1;
    options.num_threads                  = 8;
    options.minimizer_progress_to_stdout = false;
  }

  void prepareConstraints(const VertexList::Vector& sourcePoints, const VertexList::Vector& targetPoints,
                          const VertexList::Vector& sourceNormals, const VertexList::Vector& targetNormals,
                          const MatchList& matches, const PoseIncrement<double>& poseIncrement,
                          ceres::Problem& problem) const {
    const unsigned nPoints = matches.matches.size();

    for (unsigned i = 0; i < nPoints; ++i) {
      const auto match = matches.matches[i];
      if (match.idx >= 0) {
        const auto& sourcePoint = sourcePoints[i];
        const auto& targetPoint = targetPoints[match.idx];

        if (!sourcePoint.allFinite() || !targetPoint.allFinite() || (sourcePoint - targetPoint).squaredNorm() == 0.0)
          continue;

        if (this->error_metric == ErrorMetric::PointToPoint) {
          problem.AddResidualBlock(PointToPointConstraint::create(sourcePoint, targetPoint, 1), nullptr,
                                   poseIncrement.getData());
        } else if (this->error_metric == ErrorMetric::PointToPlane) {
          const auto& targetNormal = targetNormals[match.idx];

          if (!targetNormal.allFinite()) continue;

          problem.AddResidualBlock(PointToPlaneConstraint::create(sourcePoint, targetPoint, targetNormal, this->weight),
                                   nullptr, poseIncrement.getData());
        } else if (this->error_metric == ErrorMetric::Symmetric) {
          const auto& targetNormal = targetNormals[match.idx];
          const auto& sourceNormal = sourceNormals[i];
          if (!targetNormal.allFinite() && !sourceNormal.allFinite()) continue;

          problem.AddResidualBlock(
              SymmetricConstraint::create(sourcePoint, targetPoint, sourceNormal, targetNormal, this->weight), nullptr,
              poseIncrement.getData());
        }
      }
    }
  }
};

#endif