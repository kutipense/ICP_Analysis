#include <data_loader/PlyLoader.h>
#include <data_loader/RgbdLoader.h>
#include <data_types/MatchList.h>
#include <discard/Reject.h>
#include <external/FreeImageHelper.h>
#include <external/VirtualSensor.h>
#include <matcher/NearestNeighborMatcher.h>
#include <optimizing/LMOptimizer.h>
#include <optimizing/LinearOptimizer.h>
#include <optimizing/Optimizer.h>
#include <sampler/CopySampler.h>
#include <sampler/CovarianceSampler.h>
#include <sampler/RandomSampler.h>
#include <sampler/UniformSampler.h>

#include <Eigen/Dense>
#include <filesystem>
#include <iostream>

int main() {
  VertexList::Ptr bunny;
  VertexList::Ptr bunny45;

  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun000.ply");
    bunny = loader.load();
    bunny->exportToOFF("bunnyTest.off");
  }

  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun090.ply");
    bunny45 = loader.load();
    bunny45->exportToOFF("bunny90.off");
  }

  {
    LinearOptimizer<sampler::UniformSampler, discard::Reject, matcher::NearestNeighborMatcher> optimizer{
        bunny, bunny45, ErrorMetric::PointToPlane, 25};
    Eigen::Matrix4f estimatedPose = Eigen::Matrix4f::Identity();
    optimizer.optimize(estimatedPose);

    auto                           bunny45PC = VertexList::toPCL<pcl::PointXYZ>(bunny->vertices);
    VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();

    Matrix3f rot       = estimatedPose.block(0, 0, 3, 3);
    auto     _rot      = rot.eulerAngles(0, 1, 2);
    float    toDegrees = 180 / 3.1415926;
    std::cout << "angles: " << _rot(0) << " " << _rot(1) << " " << _rot(2) << std::endl;
    std::cout << estimatedPose << std::endl;

    pcl::transformPointCloud(*bunny45PC, *out_cloud, estimatedPose);
    auto v = VertexList::fromPCL(out_cloud);
    v->exportToOFF("bunnyTransformed.off");
  }

  {
    LinearOptimizer<sampler::UniformSampler, discard::Reject, matcher::NearestNeighborMatcher> optimizer{
        bunny, bunny45, ErrorMetric::Symmetric, 25};
    Eigen::Matrix4f estimatedPose = Eigen::Matrix4f::Identity();
    optimizer.optimize(estimatedPose);

    auto                           bunny45PC = VertexList::toPCL<pcl::PointXYZ>(bunny->vertices);
    VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();

    Matrix3f rot       = estimatedPose.block(0, 0, 3, 3);
    auto     _rot      = rot.eulerAngles(0, 1, 2);
    float    toDegrees = 180 / 3.1415926;
    std::cout << "angles: " << _rot(0) << " " << _rot(1) << " " << _rot(2) << std::endl;
    std::cout << estimatedPose << std::endl;

    pcl::transformPointCloud(*bunny45PC, *out_cloud, estimatedPose);
    auto v = VertexList::fromPCL(out_cloud);
    v->exportToOFF("bunnyTransformedsym.off");
  }

  return 0;
}