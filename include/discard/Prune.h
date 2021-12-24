#ifndef __RANDOM_SAMPLER_H__
#define __RANDOM_SAMPLER_H__

#include <discard/Discard.h>
#include <pcl/filters/random_sample.h>

template <typename T>
class Prune : public Discard<T> {
 public:
  using Ptr           = std::shared_ptr<Prune<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  Prune(typename T::Ptr data_ptr, size_t point_size) : Discard<T>(data_ptr), point_size_(point_size) {}

  Prune(const Prune&) = delete;
  Prune operator=(const Prune&) = delete;

  typename T::Ptr discard() override {}

 private:
  size_t point_size_;
};

#endif