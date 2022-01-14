#include <data_loader/PlyLoader.h>

#include <Eigen/Dense>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <unordered_map>

PlyLoader::PlyLoader(const std::string& data_src) : ply_data_(data_src) {}

VertexList::Ptr PlyLoader::load() {
  auto plyFilePtr      = std::make_shared<VertexList>();
  plyFilePtr->vertices = ply_data_.getVertexPositions("vertex");

  std::unordered_map<std::string, std::string> params;
  for (size_t i = 0; i < ply_data_.objInfoComments.size(); i++) {
    std::vector<std::string> splitted;
    boost::split(splitted, ply_data_.objInfoComments[i], boost::is_any_of(" "));

    if (splitted.size() == 2) params[splitted[0]] = splitted[1];
  }

  auto gridRaw = ply_data_.getElement("range_grid").getListPropertyAnySign<size_t>("vertex_indices");

  std::vector<double>          gridParsed;
  std::vector<Eigen::Vector3f> gridVector;

  gridParsed.resize(gridRaw.size());
  gridVector.reserve(gridRaw.size());

  auto height = std::stoi(params["num_rows"]);
  auto width  = std::stoi(params["num_cols"]);

  for (size_t i = 0; i < gridRaw.size(); i++) {
    if (gridRaw[i].size() == 0) {
      gridParsed[i] = std::numeric_limits<double>::infinity();
      gridVector.push_back(Eigen::Vector3f(0, 0, 0));
    } else {
      auto  vectorIdx = gridRaw[i][0];
      auto& v         = plyFilePtr->vertices[vectorIdx];
      gridParsed[i]   = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
      gridVector.push_back(Eigen::Vector3f(v[0], v[1], v[2]));
    }
  }

  float maxDistance       = 0.1f;
  float maxDistanceHalved = maxDistance / 2;
#pragma omp parallel for
  for (int v = 1; v < height - 1; ++v) {
    for (int u = 1; u < width - 1; ++u) {
      unsigned int idx = v * width + u;

      if (!std::isfinite(gridParsed[idx])) continue;

      if (!std::isfinite(gridParsed[idx + 1]) || !std::isfinite(gridParsed[idx - 1]) ||
          !std::isfinite(gridParsed[idx + width]) || !std::isfinite(gridParsed[idx - width])) {
        if (std::isfinite(gridParsed[idx])) plyFilePtr->normals.push_back({0, 0, 0});
        continue;
      }

      const float du = 0.5f * (gridParsed[idx + 1] - gridParsed[idx - 1]);
      const float dv = 0.5f * (gridParsed[idx + width] - gridParsed[idx - width]);
      if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistanceHalved || abs(dv) > maxDistanceHalved) {
        if (std::isfinite(gridParsed[idx])) plyFilePtr->normals.push_back({0, 0, 0});
        continue;
      }

      auto pU = gridVector[idx + 1] - gridVector[idx - 1];
      auto pV = gridVector[idx - width] - gridVector[idx + width];

      auto n = pU.cross(pV);
      n.normalize();
      plyFilePtr->normals.push_back({n(0), n(1), n(2)});
    }
  }

  return plyFilePtr;
}