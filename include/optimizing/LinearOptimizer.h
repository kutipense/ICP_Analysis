#ifndef __LINEAR_OPTIMIZER_H__
#define __LINEAR_OPTIMIZER_H__

#include <ceres/rotation.h>
#include <optimizing/Helpers.h>
#include <optimizing/Optimizer.h>
#include <optimizing/linear/PointToPlane.h>
#include <optimizing/linear/Symmetric.h>
#include <pcl/common/transforms.h>

template <typename SamplerType, typename DiscardType, typename MatcherType>
class LinearOptimizer : public Optimizer<SamplerType, DiscardType, MatcherType> {
 public:
  LinearOptimizer(VertexList::Ptr& source, VertexList::Ptr& target,
                  ErrorMetric error_metric = ErrorMetric::PointToPoint, unsigned int m_nIterations = 20,
                  const float weight = 2)
      : Optimizer<SamplerType, DiscardType, MatcherType>(source, target, error_metric, m_nIterations, weight) {}

  virtual void optimize(Eigen::Matrix4f& initialPose) override {
    typename SamplerType::Ptr sampler  = std::make_shared<SamplerType>(this->source, 0.002f);
    VertexList::Ptr           sampled  = sampler->sample();
    typename SamplerType::Ptr sampler2 = std::make_shared<SamplerType>(this->target, 0.002f);
    VertexList::Ptr           sampled2 = sampler2->sample();
    typename MatcherType::Ptr matcher;

    auto sourcePC = VertexList::toPCL<pcl::PointXYZ>(this->source->vertices);

    // std::cout << sampled->vertices.size() << std::endl;

    // sampled->exportToOFF("bunnySampledICP.off");
    // sampled2->exportToOFF("bunny45SampledICP.off");

    // The initial estimate can be given as an argument.

    Eigen::AngleAxisd rollAngle(0.5, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0.2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(-0.5, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    Eigen::Matrix4f fakeTr         = Eigen::Matrix4f::Identity();
    fakeTr.block(0, 0, 3, 3)       = rotationMatrix.cast<float>();
    fakeTr.block(0, 3, 3, 1)       = Eigen::Vector3f(0.0, .0, -0.0);

    sampled2->vertices = this->transformPoints(sampled->vertices, fakeTr);
    sampled2->normals  = this->transformNormals(sampled->normals, fakeTr);
    sampled2->exportToOFF("bunnyFake.off");

    Eigen::Matrix4f estimatedPose = initialPose;

    clock_t begin = clock();
    for (size_t i = 0; i < this->m_nIterations; ++i) {
      auto transformedPoints  = this->transformPoints(sampled->vertices, estimatedPose);
      auto transformedNormals = this->transformNormals(sampled->normals, estimatedPose);
      matcher                 = std::make_shared<MatcherType>(transformedPoints, sampled2->vertices);
      auto matches            = matcher->match();
      // pruneCorrespondences(transformedNormals, target.getNormals(), matches);

      VertexList::Vector sourcePoints;
      VertexList::Vector sourceNormals;
      VertexList::Vector targetPoints;
      VertexList::Vector targetNormals;

      // Add all matches to the sourcePoints and targetPoints vector,
      // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
      // the matches vector holds the index of the matching target point.
      for (size_t j = 0; j < transformedPoints.size(); j++) {
        const auto& match = matches->matches[j];
        if (match.idx >= 0) {
          sourcePoints.push_back(transformedPoints[j]);
          sourceNormals.push_back(transformedNormals[j]);

          targetPoints.push_back(sampled2->vertices[match.idx]);
          targetNormals.push_back(sampled2->normals[match.idx]);
        }
      }

      // Estimate the new pose
      if (this->error_metric == ErrorMetric::PointToPlane) {
        linear::PointToPlane p2plane(sourcePoints, targetPoints, targetNormals);
        estimatedPose = p2plane() * estimatedPose;
      } else if (this->error_metric == ErrorMetric::PointToPoint) {
        estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
      } else if (this->error_metric == ErrorMetric::Symmetric) {
        linear::Symmetric sl(sourcePoints, targetPoints, sourceNormals, targetNormals);
        estimatedPose = sl() * estimatedPose;
      }

      // std::cout << "Optimization iteration done." << std::endl;

      // VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();
      // pcl::transformPointCloud(*sourcePC, *out_cloud, estimatedPose);
      // auto v = VertexList::fromPCL(out_cloud);
      // v->exportToOFF("bunny90ICP_" + std::to_string(i) + ".off");
    }

    clock_t end         = clock();
    double  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

    // Store result
    initialPose = estimatedPose;
  }

 private:
  Matrix4f estimatePosePointToPoint(const VertexList::Vector& sourcePoints, const VertexList::Vector& targetPoints) {
    ProcrustesAligner procrustAligner;
    Matrix4f          estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

    return estimatedPose;
  }
};

#endif