#include <discard/Reject.h>

namespace discard {
Reject::Reject(Discard::DataPtr source_data, Discard::DataPtr target_data, MatchPtr match_list)
    : Discard(source_data, target_data, match_list) {}

void Reject::setMaxDistance(double distance) { dist_ = distance * distance; }
void Reject::setMaxAngle(double angle) { ang_ = angle; }

Reject::MatchPtr Reject::discard() {
  MatchPtr matches_pruned = std::make_shared<MatchType>();
  matches_pruned->matches.reserve(this->match_list_->matches.size());

  for (size_t i = 0; i < this->source_data_->vertices.size(); i++) {
    auto ind = this->match_list_->matches[i].idx;
    if (ind >= 0 && dist_) {
      auto& src_point    = this->source_data_->vertices[i];
      auto& target_point = this->target_data_->vertices[ind];
      if (sqr_dist(src_point, target_point) > *dist_) ind = -1;
    }

    if (ind >= 0 && ang_) {
      auto& src_normal    = this->source_data_->normals[i];
      auto& target_normal = this->target_data_->normals[ind];
      if (ang_between(src_normal, target_normal) > *ang_) ind = -1;
    }

    matches_pruned->matches.push_back({ind, 1.0});
  }

  return matches_pruned;
}
}  // namespace discard