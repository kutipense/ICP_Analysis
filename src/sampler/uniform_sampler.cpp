#include <pcl/filters/extract_indices.h>
#include <sampler/UniformSampler.h>

namespace sampler {

UniformSampler::UniformSampler(DataPtr data_ptr, float search_radius)
    : Sampler(data_ptr), search_radius_(search_radius) {}

UniformSampler::DataPtr UniformSampler::sample() {
  PointCloudXYZ::Ptr inputCloud    = DataType::toPCL<pcl::PointXYZ>(this->data_->vertices);
  PointCloudXYZ::Ptr inputNormals  = DataType::toPCL<pcl::PointXYZ>(this->data_->normals);
  PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

  pcl::UniformSampling<pcl::PointXYZ> filter(true);
  filter.setInputCloud(inputCloud);
  filter.setRadiusSearch(search_radius_);
  filter.filter(*filteredCloud);

  const auto indices = filter.getRemovedIndices();

  UniformSampler::DataPtr vertex_list = std::make_shared<DataType>();

  vertex_list->vertices.reserve(filteredCloud->size());
  vertex_list->normals.reserve(filteredCloud->size());
  for (auto it = filteredCloud->begin(); it != filteredCloud->end(); ++it)
    vertex_list->vertices.emplace_back(it->x, it->y, it->z);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  auto                               outputNormals = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  extract.setInputCloud(inputNormals);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(*outputNormals);

  for (auto it = outputNormals->begin(); it != outputNormals->end(); ++it)
    vertex_list->normals.emplace_back(it->x, it->y, it->z);

  return vertex_list;
}
};  // namespace sampler