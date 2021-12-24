#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <memory>
#include <data_types/VertexList.h>
#include <data_types/MatchList.h>

template <typename T, typename std::enable_if<std::is_same<MatchList, T>::value, T>::type* = nullptr>
class Matcher {
 public:
  using Ptr     = std::shared_ptr<Matcher<T>>;
  using DataPtr = VertexList::Ptr;
  using OutPtr  = typename T::Ptr;

  Matcher(DataPtr source_data, DataPtr target_data) : source_data_(source_data), target_data_(target_data) {}
  virtual ~Matcher() = default;

  // return new ptr
  virtual OutPtr match() = 0;

 protected:
  DataPtr source_data_;
  DataPtr target_data_;
};

#endif