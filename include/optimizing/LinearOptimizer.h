#ifndef __LINEAR_OPTIMIZER_H__
#define __LINEAR_OPTIMIZER_H__

#include <ceres/rotation.h>
#include <optimizing/Helpers.h>
#include <optimizing/Optimizer.h>
#include <optimizing/linear/PointToPlane.h>
#include <optimizing/linear/PointToPoint.h>
#include <optimizing/linear/Symmetric.h>
#include <pcl/common/transforms.h>

class LinearOptimizer : public Optimizer {
 public:
  using Optimizer::Optimizer;
  void optimize(Eigen::Matrix4f& initialPose) override;
};

#endif