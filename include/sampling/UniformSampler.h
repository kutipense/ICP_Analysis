#ifndef __UNIFORM_SAMPLER_H__
#define __UNIFORM_SAMPLER_H__

#include <pcl/filters/uniform_sampling.h>
#include <sampling/Sampler.h>

template <typename T>
class UniformSampler : public Sampler<T> {
 public:
  using Ptr           = std::shared_ptr<UniformSampler<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  UniformSampler(typename T::Ptr data_ptr, float search_radius) : Sampler<T>(data_ptr), search_radius_(search_radius) {}

  UniformSampler(const UniformSampler&) = delete;
  UniformSampler operator=(const UniformSampler&) = delete;

  typename T::Ptr sample() override {
    PointCloudXYZ::Ptr inputCloud    = this->data_->toPCL();
    PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(inputCloud);
    filter.setRadiusSearch(search_radius_);
    filter.filter(*filteredCloud);

    return T::fromPCL(filteredCloud);
  }

 private:
  float search_radius_;
};

#endif