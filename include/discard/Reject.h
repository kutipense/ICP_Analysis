#ifndef __REJECT_H__
#define __REJECT_H__

#include <discard/Discard.h>

#include <optional>

template <typename T, typename M>
class Reject : public Discard<T, M> {
 public:
  using Ptr           = std::shared_ptr<Reject<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  Reject(typename T::Ptr DataPtr source_data, typename T::Ptr target_data)
      : Discard<T>(source_data, target_data), point_size_(point_size) {}

  Reject(const Reject&) = delete;
  Reject operator=(const Reject&) = delete;

  void setMaxDistance(double distance) { dist_ = distance * distance; }
  void setMaxAngle(double angle) { ang_ = angle; }

  typename M::Ptr discard() override {
    typename M::Ptr matches_pruned = std::make_shared<T>();
    matches_pruned->matches.reserve(match_list_->matches.size());

    for (size_t i = 0; i < source_data_->vertices.size(); i++) {
      auto& src_point    = source_data_->vertices[i];
      auto& target_point = target_data_->vertices[match_list_->matches[i]];

      size_t ind = i;
      if (dist_ && sqr_dist(src_point, target_point) > *dist_) ind = -1;
      if (ind >= 0 && ang_) ind = -1;

      matches_pruned.push_back({ind, 1.0});
    }

    return matches_pruned;
  }

 private:
  std::optional<double> dist_;
  std::optional<double> ang_;
};
#endif