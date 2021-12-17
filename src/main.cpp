#include <data_loader/PlyLoader.h>
#include <sampling/CopySampler.h>
#include <sampling/RandomSampler.h>
#include <sampling/UniformSampler.h>

#include <filesystem>
#include <iostream>

int main() {
  VertexList::Ptr bunny;
  VertexList::Ptr bunnyCopied;
  VertexList::Ptr bunnyUniformSampled;
  VertexList::Ptr bunnyRandomSampled;

  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun000.ply");
    bunny = loader.load();
  }

  {
    CopySampler<VertexList> sampler(bunny);
    bunnyCopied = sampler.sample();
  }

  {
    UniformSampler<VertexList> sampler(bunny, 0.005f);
    bunnyUniformSampled = sampler.sample();
  }

  {
    RandomSampler<VertexList> sampler(bunny, bunnyUniformSampled->vertices.size());
    bunnyRandomSampled = sampler.sample();
  }

  std::cout << "bunny vertex size:               " << bunny->vertices.size() << std::endl;
  std::cout << "bunnyCopied vertex size:         " << bunnyCopied->vertices.size() << std::endl;
  std::cout << "bunnyRandomSampled vertex size:  " << bunnyRandomSampled->vertices.size() << std::endl;
  std::cout << "bunnyUniformSampled vertex size: " << bunnyUniformSampled->vertices.size() << std::endl;

  bunny->exportToOFF("bunny.off");
  bunnyCopied->exportToOFF("bunnyCopied.off");
  bunnyUniformSampled->exportToOFF("bunnyUniformSampled.off");
  bunnyRandomSampled->exportToOFF("bunnyRandomSampled.off");

  return 0;
}