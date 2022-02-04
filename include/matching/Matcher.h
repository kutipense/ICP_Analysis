#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <data_types/MatchList.h>
#include <data_types/VertexList.h>

#include <memory>

template <typename T, typename std::enable_if<std::is_same<MatchList, T>::value, T>::type* = nullptr>
class Matcher {
 public:
  using Ptr      = std::shared_ptr<Matcher<T>>;
  using DataType = VertexList;
  using DataPtr  = VertexList::Ptr;
  using OutPtr   = typename T::Ptr;

  Matcher(const DataType::Vector& source_data, const DataType::Vector& target_data)
      : source_data_(source_data), target_data_(target_data) {}
  virtual ~Matcher() = default;

  // return new ptr
  virtual OutPtr match() = 0;

 protected:
  const DataType::Vector& source_data_;
  const DataType::Vector& target_data_;
};

#endif