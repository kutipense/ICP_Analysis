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

  Discard(DataPtr source_data, DataPtr target_data, MatchPtr match_list)
      : source_data_(source_data), target_data_(target_data), match_list_(match_list) {}

  virtual ~Discard() = default;

  // return new ptr
  virtual MatchPtr discard() = 0;

 protected:
  DataPtr  source_data_;
  DataPtr  target_data_;
  MatchPtr match_list_;
};
}  // namespace discard

#endif