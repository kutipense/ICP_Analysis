#ifndef __EMPTY_SAMPLER_H__
#define __EMPTY_SAMPLER_H__

#include <sampler/Sampler.h>

namespace sampler {
class CopySampler : public Sampler {
 public:
  using Ptr = std::shared_ptr<CopySampler>;

  CopySampler(DataPtr data_ptr);

  CopySampler(const CopySampler&) = delete;
  CopySampler operator=(const CopySampler&) = delete;

  DataPtr sample() override;
};
}  // namespace sampler

#endif