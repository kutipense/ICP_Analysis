#include <optimizing/LMOptimizer.h>

void LMOptimizer::optimize(Eigen::Matrix4f& initialPose) {
  if (matcher == nullptr) return;
  VertexList::Ptr src_sampled = this->source, dst_sampled = this->target;

  if (src_sampler) {
    src_sampler->setDataPtr(this->source);
    src_sampled = src_sampler->sample();
  }
  if (dst_sampler) {
    dst_sampler->setDataPtr(this->target);
    dst_sampled = dst_sampler->sample();
  }

  Eigen::Matrix4f estimatedPose = initialPose;
  double          incrementArray[6];
  auto            poseIncrement = PoseIncrement<double>(incrementArray);
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
    trSource->vertices = this->transformPoints(src_sampled->vertices, estimatedPose);
    trSource->normals  = this->transformNormals(src_sampled->normals, estimatedPose);
    auto matches       = matcher->match(trSource->vertices, dst_sampled->vertices);

    if (discarder) { matches = discarder->discard(trSource, dst_sampled, matches); }

    // Prepare point-to-point and point-to-plane constraints.
    ceres::Problem problem;
    prepareConstraints(trSource->vertices, dst_sampled->vertices, trSource->normals, dst_sampled->normals, *matches,
                       poseIncrement, problem);

    // Configure options for the solver.
    ceres::Solver::Options options;
    configureSolver(options);

    // Run the solver (for one iteration).
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
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

void LMOptimizer::configureSolver(ceres::Solver::Options& options) {
  // Ceres options.
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.use_nonmonotonic_steps     = false;
  // options.linear_solver_type           = ceres::DENSE_QR;
  options.max_num_iterations           = 1;
  options.num_threads                  = 8;
  options.minimizer_progress_to_stdout = false;
}

void LMOptimizer::prepareConstraints(const VertexList::Vector& sourcePoints, const VertexList::Vector& targetPoints,
                                     const VertexList::Vector& sourceNormals, const VertexList::Vector& targetNormals,
                                     const MatchList& matches, const PoseIncrement<double>& poseIncrement,
                                     ceres::Problem& problem) {
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