#ifndef __UNIFORM_SAMPLER_H__
#define __UNIFORM_SAMPLER_H__

#include <pcl/filters/uniform_sampling.h>
#include <sampler/Sampler.h>

namespace sampler {
class UniformSampler : public Sampler {
 public:
  using Ptr = std::shared_ptr<UniformSampler>;

  UniformSampler(float search_radius);
  VertexList::Ptr sample() override;

 private:
  float search_radius_;
};
}  // namespace sampler

#endif