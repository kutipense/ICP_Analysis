#include <data_loader/PlyLoader.h>

#include <filesystem>
#include <iostream>

int main() {
  PlyFile::Ptr bunny;

  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun000.ply");
    bunny = loader.load();
  }

  std::cout << bunny->vertices.size() << std::endl;
  bunny->exportToOFF("test.off");
  return 0;
}