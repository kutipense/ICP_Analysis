#include <sampler/RandomSampler.h>

namespace sampler {
RandomSampler::RandomSampler(DataPtr data_ptr, size_t point_size) : Sampler(data_ptr), point_size_(point_size) {}

RandomSampler::DataPtr RandomSampler::sample() {
  PointCloudXYZ::Ptr inputCloud    = DataType::toPCL<pcl::PointXYZ>(this->data_->vertices);
  PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

  pcl::RandomSample<pcl::PointXYZ> filter;
  filter.setInputCloud(inputCloud);
  filter.setSample(point_size_);
  filter.filter(*filteredCloud);

  return DataType::fromPCL(filteredCloud);
}
}  // namespace sampler