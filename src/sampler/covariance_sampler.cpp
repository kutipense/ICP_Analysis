#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <sampler/CovarianceSampler.h>

namespace sampler {
CovarianceSampler::CovarianceSampler(DataPtr data_ptr, size_t point_size)
    : Sampler(data_ptr), point_size_(point_size) {}

CovarianceSampler::DataPtr CovarianceSampler::sample() {
  PointCloudXYZ::Ptr inputCloud    = DataType::toPCL<pcl::PointXYZ>(this->data_->vertices);
  PointCloudXYZ::Ptr inputNormals  = DataType::toPCL<pcl::PointXYZ>(this->data_->normals);
  PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = DataType::toPCL<pcl::Normal>(this->data_->normals);

  pcl::CovarianceSampling<pcl::PointXYZ, pcl::Normal> filter;
  filter.setInputCloud(inputCloud);
  filter.setNumberOfSamples(point_size_);
  filter.setNormals(cloud_normals);
  filter.filter(*filteredCloud);

  const auto indices = filter.getRemovedIndices();

  CovarianceSampler::DataPtr vertex_list = std::make_shared<DataType>();

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
}  // namespace sampler