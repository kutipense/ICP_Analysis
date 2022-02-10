#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <data_types/MatchList.h>
#include <data_types/VertexList.h>

#include <memory>

namespace matcher {
class Matcher {
 public:
  using Ptr      = std::shared_ptr<Matcher>;
  using DataType = VertexList;
  using DataPtr  = VertexList::Ptr;
  using OutPtr   = MatchList::Ptr;

  virtual ~Matcher() = default;

  // return new ptr
  virtual OutPtr match(const VertexList::Vector& source_data, const VertexList::Vector& target_data) = 0;
};
}  // namespace matcher

#endif