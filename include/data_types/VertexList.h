#ifndef __VERTEX_LIST_H__
#define __VERTEX_LIST_H__

#include <fstream>
#include <memory>
#include <vector>

struct VertexList {
  using Ptr    = std::shared_ptr<VertexList>;
  using Vector = std::vector<std::array<double, 3>>;

  VertexList::Vector vertices;

  bool exportToOFF(const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    outFile << "COFF" << std::endl
            << "# numVertices numFaces numEdges" << std::endl
            << vertices.size() << " 0 0" << std::endl
            << "# list of vertices" << std::endl
            << "# X Y Z R G B" << std::endl;

    for (auto& p : vertices) outFile << p[0] << " " << p[1] << " " << p[2] << " 255 255 255" << std::endl;

    outFile.close();
    return true;
  }
};

#endif