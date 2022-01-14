#ifndef __SURFACE_NORMAL_REJECT_H__
#define __SURFACE_NORMAL_REJECT_H__

#include <discard/Discard.h>

#include <optional>

template <typename T, typename M>
class SurfaceNormalReject : public Discard<T, M> {
 public:
  using Ptr           = std::shared_ptr<SurfaceNormalReject<T, M>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  SurfaceNormalReject(typename T::Ptr source_data, typename T::Ptr target_data, typename M::Ptr match_list)
      : Discard<T, M>(source_data, target_data, match_list) {}

  SurfaceNormalReject(const SurfaceNormalReject&) = delete;
  SurfaceNormalReject operator=(const SurfaceNormalReject&) = delete;

  void setMaxAngle(double angle) { ang_ = angle; }

  typename M::Ptr discard() override {
    auto inputPc      = T::toPCL<pcl::PointXYZ>(this->source_data->vertices);
    auto targetPc     = T::toPCL<pcl::PointXYZ>(this->target_data_->vertices);
    auto inputNormals = T::toPCL<pcl::PointXYZ>(this->target_data_->vertices);

    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej(
        new pcl::registration::CorrespondenceRejectorSurfaceNormal);

    rej->setInputSource(this->source_data_->vertices);
    rej->setInputNormals(this->source_data_->normals);
    rej->setInputTarget(this->target_data_->vertices);
    rej->setTargetNormals(this->target_data_->normals);
    rej->setThreshold(ang_ != nullptr ? ang_.value() : 0);
    rej->setInputCorrespondences(remaining_correspondences_temp);
    rej->getCorrespondences(remaining_correspondences);

    pcl::Correspondences all_correspondences;
    rej->determineCorrespondences(all_correspondences);

    typename M::Ptr matches_pruned = std::make_shared<M>();
    matches_pruned->matches.reserve(all_correspondences.size());
    for (size_t i = 0; i < all_correspondences.size(); i++)
      matches_pruned->matches.push_back({all_correspondences[i].index_match, 1.0});

    return matches_pruned;
  }

 private:
  std::optional<double> dist_;
  std::optional<double> ang_;
};

#endif