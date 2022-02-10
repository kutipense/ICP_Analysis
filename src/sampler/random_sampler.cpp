#include <sampler/RandomSampler.h>

namespace sampler {
RandomSampler::RandomSampler(size_t point_size) : point_size_(point_size) {}

VertexList::Ptr RandomSampler::sample() {
  PointCloudXYZ::Ptr inputCloud    = VertexList::toPCL<pcl::PointXYZ>(this->data_->vertices);
  PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

  pcl::RandomSample<pcl::PointXYZ> filter;
  filter.setInputCloud(inputCloud);
  filter.setSample(point_size_);
  filter.filter(*filteredCloud);

  return VertexList::fromPCL(filteredCloud);
}
}  // namespace sampler