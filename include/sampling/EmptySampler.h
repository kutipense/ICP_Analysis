#ifndef __EMPTY_SAMPLER_H__
#define __EMPTY_SAMPLER_H__

template <typename T>
class EmptySampler : Sampler<T> {
  using Ptr = std::shared_ptr<EmptySampler<T>>;
};

#endif