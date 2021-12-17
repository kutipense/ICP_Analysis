#ifndef __DATA_LOADER_H__
#define __DATA_LOADER_H__

class DataLoader {
 public:
  virtual ~DataLoader() = 0;
};

inline DataLoader::~DataLoader() = default;

#endif