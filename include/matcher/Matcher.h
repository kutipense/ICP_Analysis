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

  Matcher(const DataType::Vector& source_data, const DataType::Vector& target_data)
      : source_data_(source_data), target_data_(target_data) {}
  virtual ~Matcher() = default;

  // return new ptr
  virtual OutPtr match() = 0;

 protected:
  const DataType::Vector& source_data_;
  const DataType::Vector& target_data_;
};
}  // namespace matcher

#endif