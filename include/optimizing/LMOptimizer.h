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

class LMOptimizer : public Optimizer {
 public:
  using Optimizer::Optimizer;
  void optimize(Eigen::Matrix4f& initialPose) override;

 private:
  void configureSolver(ceres::Solver::Options& options);
  void prepareConstraints(const VertexList::Vector& sourcePoints, const VertexList::Vector& targetPoints,
                          const VertexList::Vector& sourceNormals, const VertexList::Vector& targetNormals,
                          const MatchList& matches, const PoseIncrement<double>& poseIncrement,
                          ceres::Problem& problem);
};

#endif