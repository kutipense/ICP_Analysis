#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <memory>

template <typename T, typename std::enable_if<std::is_same<VertexList, T>::value, T>::type* = nullptr>
class Matcher {
 public:
  using Ptr     = std::shared_ptr<Matcher<T>>;
  using DataPtr = typename T::Ptr;

  Matcher(DataPtr source_data, DataPtr target_data) : source_data_(source_data), target_data_(target_data) {}
  virtual ~Matcher() = default;

  // return new ptr
  virtual MatchList::Ptr match() = 0;

 protected:
  DataPtr source_data_;
  DataPtr target_data_;
};

#endif