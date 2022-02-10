#ifndef __RANDOM_SAMPLER_H__
#define __RANDOM_SAMPLER_H__

#include <pcl/filters/random_sample.h>
#include <sampler/Sampler.h>

namespace sampler {
class RandomSampler : public Sampler {
 public:
  using Ptr = std::shared_ptr<RandomSampler>;

  RandomSampler(size_t point_size);
  VertexList::Ptr sample() override;

 private:
  size_t point_size_;
};
}  // namespace sampler
#endif