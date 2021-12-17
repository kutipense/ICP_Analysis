#include <data_loader/PlyLoader.h>
#include <data_loader/RgbdLoader.h>
#include <external/FreeImageHelper.h>
#include <external/VirtualSensor.h>
#include <sampling/CopySampler.h>
#include <sampling/CovarianceSampler.h>
#include <sampling/RandomSampler.h>
#include <sampling/UniformSampler.h>

#include <filesystem>
#include <iostream>

int main() {
  VertexList::Ptr bunny;
  VertexList::Ptr bunnyCopied;
  VertexList::Ptr bunnyUniformSampled;
  VertexList::Ptr bunnyRandomSampled;
  VertexList::Ptr bunnyCovarianceSampled;

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

  {
    CovarianceSampler<VertexList> sampler(bunny, bunny->vertices.size() / 5);
    bunnyCovarianceSampled = sampler.sample();
  }

  std::cout << "bunny vertex size:               " << bunny->vertices.size() << std::endl;
  std::cout << "bunnyCopied vertex size:         " << bunnyCopied->vertices.size() << std::endl;
  std::cout << "bunnyRandomSampled vertex size:  " << bunnyRandomSampled->vertices.size() << std::endl;
  std::cout << "bunnyUniformSampled vertex size: " << bunnyUniformSampled->vertices.size() << std::endl;
  std::cout << "bunnyCovarianceSampled vertex size: " << bunnyCovarianceSampled->vertices.size() << std::endl;

  bunny->exportToOFF("bunny.off");
  bunnyCopied->exportToOFF("bunnyCopied.off");
  bunnyUniformSampled->exportToOFF("bunnyUniformSampled.off");
  bunnyRandomSampled->exportToOFF("bunnyRandomSampled.off");
  bunnyCovarianceSampled->exportToOFF("bunnyCovarianceSampled.off");

  VertexList::Ptr fr1;

  {
    RgbdLoader loader("../dataset/fr1xyz/rgbd_dataset_freiburg1_xyz/");
    fr1 = loader.load();
  }

  fr1->exportToOFF("fr1.off", true);

  return 0;
}