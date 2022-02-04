#include <data_types/VertexList.h>

bool VertexList::exportToOFF(const std::string& filename, bool with_color) {
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
