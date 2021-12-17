#ifndef __PLY_LOADER_H__
#define __PLY_LOADER_H__

#include <data_loader/DataLoader.h>
#include <data_types/PlyFile.h>
#include <happly/happly.h>

#include <array>
#include <vector>

class PlyLoader : DataLoader {
 public:
  PlyLoader(const std::string&);
  PlyLoader(const PlyLoader&) = delete;
  PlyLoader operator=(const PlyLoader&) = delete;
  ~PlyLoader() = default;

  PlyFile::Ptr load();

 private:
  happly::PLYData ply_data_;
};

#endif