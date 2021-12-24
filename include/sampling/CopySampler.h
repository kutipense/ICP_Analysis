#ifndef __EMPTY_SAMPLER_H__
#define __EMPTY_SAMPLER_H__

#include <sampling/Sampler.h>

template <typename T>
class CopySampler : public Sampler<T> {
 public:
  using Ptr = std::shared_ptr<CopySampler<T>>;

  CopySampler(typename T::Ptr data_ptr) : Sampler<T>(data_ptr) {}

  CopySampler(const CopySampler&) = delete;
  CopySampler operator=(const CopySampler&) = delete;

  typename T::Ptr sample() override {
    typename T::Ptr data_copy = std::make_shared<T>();

    *data_copy = *(this->data_);

    return data_copy;
  }
};

#endif