#ifndef __UNIFORM_SAMPLER_H__
#define __UNIFORM_SAMPLER_H__

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/uniform_sampling.h>
#include <sampling/Sampler.h>

#include <iostream>

template <typename T>
class UniformSampler : public Sampler<T> {
 public:
  using Ptr           = std::shared_ptr<UniformSampler<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  UniformSampler(typename T::Ptr data_ptr, float search_radius) : Sampler<T>(data_ptr), search_radius_(search_radius) {}

  UniformSampler(const UniformSampler &) = delete;
  UniformSampler operator=(const UniformSampler &) = delete;

  typename T::Ptr sample() override {
    PointCloudXYZ::Ptr inputCloud    = VertexList::toPCL<pcl::PointXYZ>(this->data_->vertices);
    PointCloudXYZ::Ptr inputNormals  = VertexList::toPCL<pcl::PointXYZ>(this->data_->normals);
    PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

    pcl::UniformSampling<pcl::PointXYZ> filter(true);
    filter.setInputCloud(inputCloud);
    filter.setRadiusSearch(search_radius_);
    filter.filter(*filteredCloud);

    const auto indices = filter.getRemovedIndices();

    VertexList::Ptr vertex_list = std::make_shared<VertexList>();

    vertex_list->vertices.reserve(filteredCloud->size());
    vertex_list->normals.reserve(filteredCloud->size());
    for (auto it = filteredCloud->begin(); it != filteredCloud->end(); ++it)
      vertex_list->vertices.push_back({it->x, it->y, it->z});

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    auto                               outputNormals = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    extract.setInputCloud(inputNormals);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*outputNormals);

    for (auto it = outputNormals->begin(); it != outputNormals->end(); ++it)
      vertex_list->normals.push_back({it->x, it->y, it->z});

    return vertex_list;
  }

 private:
  float search_radius_;
};

#endif