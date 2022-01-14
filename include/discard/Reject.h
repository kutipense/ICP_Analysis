#ifndef __REJECT_H__
#define __REJECT_H__

#include <discard/Discard.h>

#include <optional>

template <typename T, typename M>
class Reject : public Discard<T, M> {
 public:
  using Ptr           = std::shared_ptr<Reject<T, M>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  Reject(typename T::Ptr source_data, typename T::Ptr target_data, typename M::Ptr match_list)
      : Discard<T, M>(source_data, target_data, match_list) {}

  Reject(const Reject&) = delete;
  Reject operator=(const Reject&) = delete;

  void setMaxDistance(double distance) { dist_ = distance * distance; }
  void setMaxAngle(double angle) { ang_ = angle; }

  typename M::Ptr discard() override {
    typename M::Ptr matches_pruned = std::make_shared<M>();
    matches_pruned->matches.reserve(this->match_list_->matches.size());

    for (size_t i = 0; i < this->source_data_->vertices.size(); i++) {
      auto& src_point    = this->source_data_->vertices[i];
      auto& target_point = this->target_data_->vertices[this->match_list_->matches[i].idx];

      auto ind = this->match_list_->matches[i].idx;
      if (dist_ && sqr_dist(src_point, target_point) > *dist_) ind = -1;
      if (ind >= 0 && ang_) ind = -1;

      matches_pruned->matches.push_back({ind, 1.0});
    }

    return matches_pruned;
  }

 private:
  std::optional<double> dist_;
  std::optional<double> ang_;
};
#endif