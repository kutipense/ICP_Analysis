#ifndef __DISCARD_H__
#define __DISCARD_H__

#include <data_types/MatchList.h>
#include <data_types/VertexList.h>

#include <memory>

namespace discard {
class Discard {
 public:
  using Ptr       = std::shared_ptr<Discard>;
  using DataType  = VertexList;
  using DataPtr   = VertexList::Ptr;
  using MatchType = MatchList;
  using MatchPtr  = MatchList::Ptr;

  virtual ~Discard() = default;

  // return new ptr
  virtual MatchPtr discard(DataPtr source_data, DataPtr target_data, MatchPtr match_list) = 0;
};
}  // namespace discard

#endif