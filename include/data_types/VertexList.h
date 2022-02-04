#ifndef __VERTEX_LIST_H__
#define __VERTEX_LIST_H__

// clang-format off
#include <external/Eigen.h>
// clang-format on

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <memory>

inline double sqr_dist(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) { return (p1 - p2).squaredNorm(); }

inline double ang_between(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
  return acos(p1.dot(p2) / p1.norm() / p2.norm());
}

struct VertexList {
  using Ptr              = std::shared_ptr<VertexList>;
  using Vector           = std::vector<Eigen::Vector3f>;
  using VectorByte       = std::vector<Eigen::Vector4uc>;
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

  bool exportToOFF(const std::string& filename, bool with_color = false);
};

#endif