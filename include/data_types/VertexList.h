#ifndef __VERTEX_LIST_H__
#define __VERTEX_LIST_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <memory>
#include <vector>

inline double sqr_dist(std::array<double, 3> p1, std::array<double, 3> p2) {
  return std::pow(p1[0] - p2[0], 2) + std::pow(p1[1] - p2[1], 2) + std::pow(p1[2] - p2[2], 2);
}

struct VertexList {
  using Ptr              = std::shared_ptr<VertexList>;
  using Vector           = std::vector<std::array<double, 3>>;
  using VectorByte       = std::vector<std::array<unsigned char, 4>>;
  using PointCloudXYZ    = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudNormal = pcl::PointCloud<pcl::Normal>;

  VertexList::Vector     vertices;
  VertexList::Vector     normals;
  VertexList::VectorByte colors;

  template <typename T>
  static typename pcl::PointCloud<T>::Ptr toPCL(const Vector& vec) {
    auto cloud = boost::make_shared<pcl::PointCloud<T>>();
    for (auto& p : vec) cloud->emplace_back(p[0], p[1], p[2]);
    return cloud;
  }

  static VertexList::Ptr fromPCL(PointCloudXYZ::Ptr cloud) {
    VertexList::Ptr vertex_list = std::make_shared<VertexList>();
    vertex_list->vertices.reserve(cloud->size());
    for (auto it = cloud->begin(); it != cloud->end(); ++it) vertex_list->vertices.push_back({it->x, it->y, it->z});
    return vertex_list;
  }

  bool exportToOFF(const std::string& filename, bool with_color = false) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    outFile << "COFF" << std::endl
            << "# numVertices numFaces numEdges" << std::endl
            << vertices.size() << " 0 0" << std::endl
            << "# list of vertices" << std::endl
            << "# X Y Z R G B" << std::endl;

    for (size_t i = 0; i < vertices.size(); i++) {
      auto& p = vertices[i];
      outFile << p[0] << " " << p[1] << " " << p[2];

      if (with_color) {
        auto& c = colors[i];
        outFile << " " << +c[0] << " " << +c[1] << " " << +c[2];
      }

      outFile << std::endl;
    }

    outFile.close();
    return true;
  }
};

#endif