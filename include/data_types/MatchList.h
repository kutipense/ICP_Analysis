#ifndef __MATCH_LIST_H__
#define __MATCH_LIST_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <memory>
#include <vector>

struct Match {
  size_t idx;
  float  weight;
};

struct MatchList {
  using Ptr    = std::shared_ptr<MatchList>;
  using Vector = std::vector<Match>;
  MatchList::Vector matches;
};

#endif