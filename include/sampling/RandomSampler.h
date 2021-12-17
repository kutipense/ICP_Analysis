#ifndef __RANDOM_SAMPLER_H__
#define __RANDOM_SAMPLER_H__

#include <pcl/filters/random_sample.h>
#include <sampling/Sampler.h>

template <typename T>
class RandomSampler : public Sampler<T> {
 public:
  using Ptr           = std::shared_ptr<RandomSampler<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  RandomSampler(typename T::Ptr data_ptr, size_t point_size) : Sampler<T>(data_ptr), point_size_(point_size) {}

  RandomSampler(const RandomSampler&) = delete;
  RandomSampler operator=(const RandomSampler&) = delete;

  typename T::Ptr sample() override {
    PointCloudXYZ::Ptr inputCloud    = this->data_->toPCL();
    PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

    pcl::RandomSample<pcl::PointXYZ> filter;
    filter.setInputCloud(inputCloud);
    filter.setSample(point_size_);
    filter.filter(*filteredCloud);

    return T::fromPCL(filteredCloud);
  }

 private:
  size_t point_size_;
};
#endif