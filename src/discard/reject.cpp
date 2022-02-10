#include <discard/Reject.h>

namespace discard {
void Reject::setMaxDistance(double distance) { dist_ = distance * distance; }
void Reject::setMaxAngle(double angle) { ang_ = angle; }

Reject::MatchPtr Reject::discard(DataPtr source_data, DataPtr target_data, MatchPtr match_list) {
  MatchPtr matches_pruned = std::make_shared<MatchType>();
  matches_pruned->matches.reserve(match_list->matches.size());

  for (size_t i = 0; i < source_data->vertices.size(); i++) {
    auto ind = match_list->matches[i].idx;
    if (ind >= 0 && dist_) {
      auto& src_point    = source_data->vertices[i];
      auto& target_point = target_data->vertices[ind];
      if (sqr_dist(src_point, target_point) > *dist_) ind = -1;
    }

    if (ind >= 0 && ang_) {
      auto& src_normal    = source_data->normals[i];
      auto& target_normal = target_data->normals[ind];
      if (src_normal.dot(target_normal) < 0 || ang_between(src_normal, target_normal) > *ang_) ind = -1;
    }

    matches_pruned->matches.push_back({ind, 1.0});
  }

  return matches_pruned;
}
}  // namespace discard