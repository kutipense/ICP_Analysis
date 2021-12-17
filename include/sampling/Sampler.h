#ifndef __SAMPLER_H__
#define __SAMPLER_H__

#include <memory>

template <typename T>
class Sampler {
 public:
  using Ptr     = std::shared_ptr<Sampler<T>>;
  using DataPtr = typename T::Ptr;

  Sampler(DataPtr data_ptr) : data_(data_ptr) {}
  virtual ~Sampler() = 0;

  // return new ptr
  virtual DataPtr sample() = 0;

 protected:
  DataPtr data_;
};

template <typename T>
inline Sampler<T>::~Sampler() = default;

#endif