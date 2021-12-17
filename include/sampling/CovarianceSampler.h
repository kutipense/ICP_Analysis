#ifndef __COVARIANCE_SAMPLER_H__
#define __COVARIANCE_SAMPLER_H__

#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <sampling/Sampler.h>

template <typename T>
class CovarianceSampler : public Sampler<T> {
 public:
  using Ptr           = std::shared_ptr<CovarianceSampler<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  CovarianceSampler(typename T::Ptr data_ptr, size_t point_size) : Sampler<T>(data_ptr), point_size_(point_size) {}

  CovarianceSampler(const CovarianceSampler&) = delete;
  CovarianceSampler operator=(const CovarianceSampler&) = delete;

  typename T::Ptr sample() override {
    PointCloudXYZ::Ptr inputCloud    = this->data_->toPCL();
    PointCloudXYZ::Ptr filteredCloud = boost::make_shared<PointCloudXYZ>();

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(inputCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.02);
    ne.compute(*cloud_normals);

    pcl::CovarianceSampling<pcl::PointXYZ, pcl::Normal> filter;
    filter.setInputCloud(inputCloud);
    filter.setNumberOfSamples(point_size_);
    filter.setNormals(cloud_normals);
    filter.filter(*filteredCloud);

    return T::fromPCL(filteredCloud);
  }

 private:
  size_t point_size_;
};
#endif