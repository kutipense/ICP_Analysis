#ifndef __SAMPLER_H__
#define __SAMPLER_H__

#include <data_types/VertexList.h>

#include <memory>

namespace sampler {
class Sampler {
 public:
  using Ptr      = std::shared_ptr<Sampler>;
  using DataType = VertexList;
  using DataPtr  = VertexList::Ptr;

  Sampler(DataPtr data_ptr) : data_(data_ptr) {}
  virtual ~Sampler() = default;

  // return new ptr
  virtual DataPtr sample() = 0;

 protected:
  DataPtr data_;
};
}  // namespace sampler

#endif