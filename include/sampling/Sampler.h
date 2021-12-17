#ifndef __SAMPLER_H__
#define __SAMPLER_H__

#include <memory>

template <typename T, typename std::enable_if<std::is_same<VertexList, T>::value, T>::type* = nullptr>
class Sampler {
 public:
  using Ptr     = std::shared_ptr<Sampler<T>>;
  using DataPtr = typename T::Ptr;

  Sampler(DataPtr data_ptr) : data_(data_ptr) {}
  virtual ~Sampler() = default;

  // return new ptr
  virtual DataPtr sample() = 0;

 protected:
  DataPtr data_;
};

#endif