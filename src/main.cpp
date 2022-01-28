#include <data_loader/PlyLoader.h>
#include <data_loader/RgbdLoader.h>
#include <data_types/MatchList.h>
#include <discard/Reject.h>
#include <external/FreeImageHelper.h>
#include <external/VirtualSensor.h>
#include <matching/NearestNeighborMatcher.h>
#include <optimizing/LMOptimizer.h>
#include <optimizing/LinearOptimizer.h>
#include <optimizing/Optimizer.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <sampling/CopySampler.h>
#include <sampling/CovarianceSampler.h>
#include <sampling/RandomSampler.h>
#include <sampling/UniformSampler.h>

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
    PlyLoader loader("../dataset/3dscanrep/bunny/data/bun045.ply");
    bunny45 = loader.load();
  }

  {
    LMOptimizer<VertexList, MatchList, UniformSampler<VertexList>, Reject<VertexList, MatchList>,
                NearestNeighborMatcher<MatchList>>
                    optimizer{bunny, bunny45, ErrorMetric::Symmetric};
    Eigen::Matrix4f estimatedPose = Eigen::Matrix4f::Identity();
    optimizer.optimize(estimatedPose);

    auto                           bunny45PC = VertexList::toPCL<pcl::PointXYZ>(bunny->vertices);
    VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();

    pcl::transformPointCloud(*bunny45PC, *out_cloud, estimatedPose);
    auto v = VertexList::fromPCL(out_cloud);
    v->exportToOFF("bunny45ICP.off");
  }

  return 0;
}