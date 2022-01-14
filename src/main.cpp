#include <data_loader/PlyLoader.h>
#include <data_loader/RgbdLoader.h>
#include <data_types/MatchList.h>
#include <discard/Reject.h>
#include <external/FreeImageHelper.h>
#include <external/VirtualSensor.h>
#include <matching/NearestNeighborMatcher.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <sampling/CopySampler.h>
#include <sampling/CovarianceSampler.h>
#include <sampling/RandomSampler.h>
#include <sampling/UniformSampler.h>

#include <filesystem>
#include <iostream>

int main() {
  VertexList::Ptr bunny;
  VertexList::Ptr bunny45;
  VertexList::Ptr bunnyUniformSampled;
  // VertexList::Ptr bunnyCopied;
  // VertexList::Ptr bunnyRandomSampled;
  VertexList::Ptr bunnyCovarianceSampled;
  MatchList::Ptr  bunniesMatch;
  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun000.ply");
    bunny = loader.load();
    bunny->exportToOFF("bunnyTest.off");
  }

  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun045.ply");
    bunny45 = loader.load();
  }

  {
    UniformSampler<VertexList> sampler(bunny, 0.001f);
    bunnyUniformSampled = sampler.sample();
  }

  // {
  //   RandomSampler<VertexList> sampler(bunny, bunnyUniformSampled->vertices.size());
  //   bunnyRandomSampled = sampler.sample();
  // }

  // {
  //   CovarianceSampler<VertexList> sampler(bunny, bunny->vertices.size() / 5);
  //   bunnyCovarianceSampled = sampler.sample();
  //   // bunnyCovarianceSampled->exportToOFF("bunnyCovarianceSampled.off");
  // }
  // bunny->exportToOFF("bunnyTest2.off");

  // std::cout << "bunny uniform sampled vertex size: " << bunnyUniformSampled->vertices.size() << std::endl;
  // std::cout << "bunny45 vertex size:               " << bunny45->vertices.size() << std::endl;
  // // std::cout << "bunnyCopied vertex size:         " << bunnyCopied->vertices.size() << std::endl;
  // // std::cout << "bunnyRandomSampled vertex size:  " << bunnyRandomSampled->vertices.size() << std::endl;
  // // std::cout << "bunnyUniformSampled vertex size: " << bunnyUniformSampled->vertices.size() << std::endl;
  // // std::cout << "bunnyCovarianceSampled vertex size: " << bunnyCovarianceSampled->vertices.size() << std::endl;
  {
    NearestNeighborMatcher<MatchList> matcher(bunnyUniformSampled, bunny45);
    bunniesMatch = matcher.match();
  }

  // std::cout << "bunniesMatch vertex size: " << bunniesMatch->matches.size() << std::endl;

  {
    Reject<VertexList, MatchList> reject(bunnyUniformSampled, bunny45, bunniesMatch);
    reject.setMaxDistance(0.005);
    // reject.setMaxAngle(M_PI / 3);

    auto matchesPruned = reject.discard();

    VertexList::Ptr bunnyUniformSampledPruned = std::make_shared<VertexList>();
    VertexList::Ptr bunny45Pruned             = std::make_shared<VertexList>();

    for (size_t i = 0; i < matchesPruned->matches.size(); i++) {
      if (matchesPruned->matches[i].idx < 0) continue;
      bunnyUniformSampledPruned->vertices.push_back(bunnyUniformSampled->vertices[i]);
      bunny45Pruned->vertices.push_back(bunny45->vertices[matchesPruned->matches[i].idx]);
    }

    bunnyUniformSampledPruned->exportToOFF("bunnyUniformSampledPruned.off");
    bunny45Pruned->exportToOFF("bunny45Pruned.off");
    // bunnyUniformSampled->exportToOFF("bunnyUniformSampled.off");
  }

  // bunny->exportToOFF("bunny.off");
  // bunnyCopied->exportToOFF("bunnyCopied.off");
  // bunnyUniformSampled->exportToOFF("bunnyUniformSampled.off");
  // bunnyRandomSampled->exportToOFF("bunnyRandomSampled.off");
  // bunnyCovarianceSampled->exportToOFF("bunnyCovarianceSampled.off");

  // {
  //   pcl::PointCloud<pcl::PointNormal>::Ptr source, target;
  //   // ... read or fill in source and target
  //   pcl::registration::CorrespondenceEstimationNormalShooting <pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> est;
  //   auto inputPc  = bunnyUniformSampled->toPCL();
  //   auto targetPc = bunny45->toPCL();

  //   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //   ne.setInputCloud(inputPc);
  //   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  //   ne.setSearchMethod(tree);
  //   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  //   ne.setRadiusSearch(0.02);
  //   ne.compute(*cloud_normals);

  //   est.setInputSource(inputPc);
  //   est.setSourceNormals(cloud_normals);

  //   est.setInputTarget(targetPc);

  //   // Test the first 10 correspondences for each point in source, and return the best
  //   est.setKSearch(10);

  //   pcl::Correspondences all_correspondences;
  //   // Determine all correspondences
  //   est.determineCorrespondences(all_correspondences);

  //   VertexList::Ptr bunnyUniformSampledPrunedNormal = std::make_shared<VertexList>();
  //   VertexList::Ptr bunny45PrunedNormal             = std::make_shared<VertexList>();
  //   for (size_t i = 0; i < all_correspondences.size(); i++) {
  //     if (all_correspondences[i].index_match < 0) continue;
  //     bunnyUniformSampledPrunedNormal->vertices.push_back(bunnyUniformSampled->vertices[i]);
  //     bunny45PrunedNormal->vertices.push_back(bunny45->vertices[all_correspondences[i].index_match]);
  //   }
  //   bunnyUniformSampledPrunedNormal->exportToOFF("bunnyUniformSampledPrunedNormal.off");
  //   bunny45PrunedNormal->exportToOFF("bunny45PrunedNormal.off");
  // }

  // {
  //   pcl::PointCloud<pcl::PointNormal>::Ptr source, target;
  //   // ... read or fill in source and target

  //   pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej(
  //       new pcl::registration::CorrespondenceRejectorSurfaceNormal);
  //   rej->setThreshold(0);  // Could be a lot of rotation -- just make sure they're at least within 0 degrees

  //   auto inputPc = bunnyUniformSampled->toPCL();

  //   auto targetPc = bunny45->toPCL();

  //   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  //   ne.setInputCloud(inputPc);
  //   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  //   ne.setSearchMethod(tree);
  //   pcl::PCLPointCloud2::Ptr cloud_normals(new pcl::PCLPointCloud2);
  //   ne.setRadiusSearch(0.02);
  //   ne.compute(*cloud_normals);

  //   rej->setInputSource<pcl::PointXYZ>(inputPc);
  //   rej->setSourceNormals(cloud_normals);

  //   est.setInputTarget(targetPc);

  //   // Test the first 10 correspondences for each point in source, and return the best
  //   est.setKSearch(10);

  //   pcl::Correspondences all_correspondences;
  //   // Determine all correspondences
  //   est.determineCorrespondences(all_correspondences);

  //   VertexList::Ptr bunnyUniformSampledPrunedNormal = std::make_shared<VertexList>();
  //   VertexList::Ptr bunny45PrunedNormal             = std::make_shared<VertexList>();
  //   for (size_t i = 0; i < all_correspondences.size(); i++) {
  //     if (all_correspondences[i].index_match < 0) continue;
  //     bunnyUniformSampledPrunedNormal->vertices.push_back(bunnyUniformSampled->vertices[i]);
  //     bunny45PrunedNormal->vertices.push_back(bunny45->vertices[all_correspondences[i].index_match]);
  //   }
  //   bunnyUniformSampledPrunedNormal->exportToOFF("bunnyUniformSampledPrunedNormal.off");
  //   bunny45PrunedNormal->exportToOFF("bunny45PrunedNormal.off");
  // }

  // VertexList::Ptr fr1;

  // {
  //   RgbdLoader loader("../dataset/fr1xyz/rgbd_dataset_freiburg1_xyz/");
  //   fr1 = loader.load();
  // }

  // fr1->exportToOFF("fr1.off", true);

  return 0;
}