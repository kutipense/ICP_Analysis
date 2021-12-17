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
    PointCloudXYZ::Ptr inputCloud    = boost::make_shared<PointCloudXYZ>();
    PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

    for (auto& p : this->data_->vertices) inputCloud->emplace_back(p[0], p[1], p[2]);

    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(inputCloud);
    filter.setRadiusSearch(search_radius_);
    filter.filter(*filteredCloud);

    typename T::Ptr output = std::make_shared<T>();
    output->vertices.reserve(filteredCloud->size());

    for (auto it = filteredCloud->begin(); it != filteredCloud->end(); ++it)
      output->vertices.push_back({it->x, it->y, it->z});

    return output;
  }

 private:
  float search_radius_;
};

#endif