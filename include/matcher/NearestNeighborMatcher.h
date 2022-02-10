#ifndef __NEAREST_NEIGHBOR_MATCHER_H__
#define __NEAREST_NEIGHBOR_MATCHER_H__

#include <matcher/Matcher.h>

namespace matcher {
class NearestNeighborMatcher : public Matcher {
 public:
  using Ptr           = std::shared_ptr<NearestNeighborMatcher>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  NearestNeighborMatcher(int number_of_checks = 16, float max_distance = 1, int tree_size = 1);
  MatchList::Ptr match(const VertexList::Vector& source_data, const VertexList::Vector& target_data) override;

 private:
  int   num_of_checks_, tree_size_;
  float max_distance_;
};
}  // namespace matcher
#endif