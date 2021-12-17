#ifndef __PLY_LOADER_H__
#define __PLY_LOADER_H__

#include <data_loader/VertexLoader.h>
#include <happly/happly.h>

#include <array>
#include <vector>

class PlyLoader : VertexLoader {
 public:
  PlyLoader(const std::string&);
  PlyLoader(const PlyLoader&) = delete;
  PlyLoader operator=(const PlyLoader&) = delete;
  ~PlyLoader()                          = default;

  VertexList::Ptr load() override;

 private:
  happly::PLYData ply_data_;
};

#endif