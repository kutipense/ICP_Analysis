#include <data_loader/PlyLoader.h>

#include <memory>

PlyLoader::PlyLoader(const std::string& data_src) : ply_data_(data_src) {}

VertexList::Ptr PlyLoader::load() {
  auto plyFilePtr      = std::make_shared<VertexList>();
  plyFilePtr->vertices = ply_data_.getVertexPositions("vertex");
  return plyFilePtr;
}