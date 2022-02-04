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
  VertexList::Ptr bunnySrc;
  VertexList::Ptr bunnyDst;

  {
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun000.ply");
    bunnySrc = loader.load();
    bunnySrc->exportToOFF("bunnySrc.off");
  }

  {
    bunnyDst = std::make_shared<VertexList>();
    Eigen::AngleAxisd rot_z(1.5707963267948966, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rot_y(0., Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_x(0., Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rot_z * rot_y * rot_x;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    Eigen::Matrix4f fakeTr         = Eigen::Matrix4f::Identity();
    fakeTr.block(0, 0, 3, 3)       = rotationMatrix.cast<float>();
    fakeTr.block(0, 3, 3, 1)       = Eigen::Vector3f(0.0, .0, 0.0);

    bunnyDst->vertices = Eigen::transformPoints(bunnySrc->vertices, fakeTr);
    bunnyDst->normals  = Eigen::transformNormals(bunnySrc->normals, fakeTr);
    bunnyDst->exportToOFF("bunnyDst.off");
  }

  {
    LinearOptimizer<sampler::UniformSampler, discard::Reject, matcher::NearestNeighborMatcher> optimizer{
        bunnySrc, bunnyDst, ErrorMetric::PointToPlane, 100};
    Eigen::Matrix4f estimatedPose = Eigen::Matrix4f::Identity();
    optimizer.optimize(estimatedPose);

    auto                           bunnyPC   = VertexList::toPCL<pcl::PointXYZ>(bunnySrc->vertices);
    VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();

    Matrix3f rot       = estimatedPose.block(0, 0, 3, 3);
    auto     _rot      = rot.eulerAngles(0, 1, 2);
    float    toDegrees = 180 / 3.1415926;
    std::cout << "angles: " << _rot(0) << " " << _rot(1) << " " << _rot(2) << std::endl;
    std::cout << estimatedPose << std::endl;

    pcl::transformPointCloud(*bunnyPC, *out_cloud, estimatedPose);
    auto v = VertexList::fromPCL(out_cloud);
    v->exportToOFF("bunnyTransformed.off");
  }

  {
    LinearOptimizer<sampler::UniformSampler, discard::Reject, matcher::NearestNeighborMatcher> optimizer{
        bunnySrc, bunnyDst, ErrorMetric::Symmetric, 100};
    Eigen::Matrix4f estimatedPose = Eigen::Matrix4f::Identity();
    optimizer.optimize(estimatedPose);

    auto                           bunnyPC   = VertexList::toPCL<pcl::PointXYZ>(bunnySrc->vertices);
    VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();

    Matrix3f rot       = estimatedPose.block(0, 0, 3, 3);
    auto     _rot      = rot.eulerAngles(0, 1, 2);
    float    toDegrees = 180 / 3.1415926;
    std::cout << "angles: " << _rot(0) << " " << _rot(1) << " " << _rot(2) << std::endl;
    std::cout << estimatedPose << std::endl;

    pcl::transformPointCloud(*bunnyPC, *out_cloud, estimatedPose);
    auto v = VertexList::fromPCL(out_cloud);
    v->exportToOFF("bunnyTransformedsym.off");
  }

  return 0;
}