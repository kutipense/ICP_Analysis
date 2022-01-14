#ifndef __DISCARD_H__
#define __DISCARD_H__

#include <data_types/MatchList.h>
#include <data_types/VertexList.h>

#include <memory>

template <typename T, typename M, typename std::enable_if<std::is_same<VertexList, T>::value, T>::type* = nullptr,
          typename std::enable_if<std::is_same<MatchList, M>::value, M>::type* = nullptr>
class Discard {
 public:
  using Ptr      = std::shared_ptr<Discard<T, M>>;
  using DataPtr  = typename T::Ptr;
  using MatchPtr = typename M::Ptr;

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

#endif