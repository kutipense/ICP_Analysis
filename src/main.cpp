#include <data_loader/PlyLoader.h>
#include <sampling/UniformSampler.h>

#include <filesystem>
#include <iostream>

int main() {
  VertexList::Ptr bunny;
  VertexList::Ptr bunnyUniformSampled;

  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun000.ply");
    bunny = loader.load();
  }

  {
    UniformSampler<VertexList> sampler(bunny, 0.005f);
    bunnyUniformSampled = sampler.sample();
  }

  std::cout << bunnyUniformSampled->vertices.size() << std::endl;
  bunnyUniformSampled->exportToOFF("test2.off");
  return 0;
}