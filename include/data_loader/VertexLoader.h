#ifndef __DATA_LOADER_H__
#define __DATA_LOADER_H__

#include <data_types/VertexList.h>

class VertexLoader {
 public:
  virtual ~VertexLoader()        = 0;
  virtual VertexList::Ptr load() = 0;
};

inline VertexLoader::~VertexLoader() = default;

#endif