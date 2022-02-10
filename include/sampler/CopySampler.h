#ifndef __EMPTY_SAMPLER_H__
#define __EMPTY_SAMPLER_H__

#include <sampler/Sampler.h>

namespace sampler {
class CopySampler : public Sampler {
 public:
  using Ptr = std::shared_ptr<CopySampler>;

  VertexList::Ptr sample() override;
};
}  // namespace sampler

#endif