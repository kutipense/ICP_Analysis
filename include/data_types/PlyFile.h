#include <fstream>
#include <memory>
#include <vector>

struct PlyFile {
  using Ptr = std::shared_ptr<PlyFile>;
  using Vector = std::vector<std::array<double, 3>>;

  PlyFile::Vector vertices;

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