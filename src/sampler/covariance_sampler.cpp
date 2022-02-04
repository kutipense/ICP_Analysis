#include <pcl/features/normal_3d.h>
#include <sampler/CovarianceSampler.h>

namespace sampler {
CovarianceSampler::CovarianceSampler(DataPtr data_ptr, size_t point_size)
    : Sampler(data_ptr), point_size_(point_size) {}

CovarianceSampler::DataPtr CovarianceSampler::sample() {
  PointCloudXYZ::Ptr inputCloud    = DataType::toPCL<pcl::PointXYZ>(this->data_->vertices);
  PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = DataType::toPCL<pcl::Normal>(this->data_->normals);

  pcl::CovarianceSampling<pcl::PointXYZ, pcl::Normal> filter;
  filter.setInputCloud(inputCloud);
  filter.setNumberOfSamples(point_size_);
  filter.setNormals(cloud_normals);
  filter.filter(*filteredCloud);

  return DataType::fromPCL(filteredCloud);
}
}  // namespace sampler