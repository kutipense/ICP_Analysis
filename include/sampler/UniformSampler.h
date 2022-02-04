#ifndef __UNIFORM_SAMPLER_H__
#define __UNIFORM_SAMPLER_H__

#include <pcl/filters/uniform_sampling.h>
#include <sampler/Sampler.h>

namespace sampler {
class UniformSampler : public Sampler {
 public:
  using Ptr           = std::shared_ptr<UniformSampler>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  UniformSampler(DataPtr data_ptr, float search_radius);

  UniformSampler(const UniformSampler &) = delete;
  UniformSampler operator=(const UniformSampler &) = delete;

  DataPtr sample() override;

 private:
  float search_radius_;
};
}  // namespace sampler

#endif