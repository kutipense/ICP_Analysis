#include <memory>
#include <vector>

struct PlyFile {
  using Ptr = std::shared_ptr<PlyFile>;
  using Vector = std::vector<std::array<double, 3>>;

  PlyFile::Vector vertices;

  static exportToOFF(const PlyFile&) {}
};