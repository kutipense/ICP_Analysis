#ifndef __COVARIANCE_SAMPLER_H__
#define __COVARIANCE_SAMPLER_H__

#include <pcl/filters/covariance_sampling.h>
#include <sampler/Sampler.h>

namespace sampler {
class CovarianceSampler : public Sampler {
 public:
  using Ptr              = std::shared_ptr<CovarianceSampler>;
  using PointCloudNormal = pcl::PointCloud<pcl::Normal>;

  CovarianceSampler(size_t point_size);
  VertexList::Ptr sample() override;

 private:
  size_t point_size_;
};
}  // namespace sampler
#endif