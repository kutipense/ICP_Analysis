#ifndef __SAMPLER_H__
#define __SAMPLER_H__

#include <data_types/VertexList.h>

#include <memory>

namespace sampler {
class Sampler {
 public:
  using Ptr           = std::shared_ptr<Sampler>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  Sampler() : data_(nullptr) {}
  virtual ~Sampler() = default;

  // return new ptr
  virtual VertexList::Ptr sample() = 0;

  void setDataPtr(VertexList::Ptr _ptr) { data_ = _ptr; }

 protected:
  VertexList::Ptr data_;
};
}  // namespace sampler

#endif