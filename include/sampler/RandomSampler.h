#ifndef __RANDOM_SAMPLER_H__
#define __RANDOM_SAMPLER_H__

#include <pcl/filters/random_sample.h>
#include <sampler/Sampler.h>

namespace sampler {
class RandomSampler : public Sampler {
 public:
  using Ptr           = std::shared_ptr<RandomSampler>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  RandomSampler(DataPtr data_ptr, size_t point_size);

  RandomSampler(const RandomSampler&) = delete;
  RandomSampler operator=(const RandomSampler&) = delete;

  DataPtr sample() override;

 private:
  size_t point_size_;
};
}  // namespace sampler
#endif