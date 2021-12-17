#ifndef __RANDOM_SAMPLER_H__
#define __RANDOM_SAMPLER_H__

template <typename T>
class RandomSampler : Sampler<T> {
  using Ptr = std::shared_ptr<RandomSampler<T>>;
};

#endif