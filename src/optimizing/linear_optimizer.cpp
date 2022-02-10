#include <optimizing/LinearOptimizer.h>

void LinearOptimizer::optimize(Eigen::Matrix4f& initialPose) {
  if (matcher == nullptr) return;
  VertexList::Ptr src_sampled = this->source, dst_sampled = this->target;

  if (src_sampler) {
    src_sampler->setDataPtr(this->source);
    src_sampled = src_sampler->sample();
    src_sampled->exportToOFF("srcSampled.off");
  }
  if (dst_sampler) {
    dst_sampler->setDataPtr(this->target);
    dst_sampled = dst_sampler->sample();
    dst_sampled->exportToOFF("dstSampled.off");
  }

  std::cout << src_sampled->vertices.size() << " " << dst_sampled->vertices.size() << std::endl;

  // auto sourcePC = VertexList::toPCL<pcl::PointXYZ>(this->source->vertices);

  Eigen::Matrix4f estimatedPose = initialPose;
  clock_t         begin         = clock();

  double prev_rms = 0.0;

  for (size_t i = 0; i < this->m_nIterations; ++i) {
    // if (i % 50 == 0) {
    //   auto                           bunnyPC   = VertexList::toPCL<pcl::PointXYZ>(src_sampled->vertices);
    //   VertexList::PointCloudXYZ::Ptr out_cloud = boost::make_shared<VertexList::PointCloudXYZ>();
    //   pcl::transformPointCloud(*bunnyPC, *out_cloud, estimatedPose);
    //   auto v = VertexList::fromPCL(out_cloud);
    //   v->exportToOFF("bunnyTransformed_" + std::to_string(i) + ".off");
    // }

    auto trSource      = std::make_shared<VertexList>();
    trSource->vertices = this->transformPoints(src_sampled->vertices, estimatedPose);
    trSource->normals  = this->transformNormals(src_sampled->normals, estimatedPose);

    VertexList::Vector trVec = this->transformPoints(this->source->vertices, estimatedPose);
    double             error = rms_error(trVec, this->target->vertices);
    std::cout << error << ", ";
    // std::cout << "rms_error(" << i << "): " << error << std::endl;
    // if (std::abs(error - prev_rms) < 1e-5) break;
    // prev_rms = error;

    auto matches = matcher->match(trSource->vertices, dst_sampled->vertices);
    if (discarder) { matches = discarder->discard(trSource, dst_sampled, matches); }

    // if (i == 0 || i == 50) {
    //   auto matchedSrc = std::make_shared<VertexList>();
    //   auto matchedDst = std::make_shared<VertexList>();
    //   for (size_t i = 0; i < trSource->vertices.size(); i++) {
    //     auto ind = matches->matches[i].idx;
    //     if (ind == -1) continue;

    //     matchedSrc->vertices.emplace_back(trSource->vertices[i]);
    //     matchedDst->vertices.emplace_back(dst_sampled->vertices[ind]);
    //   }
    //   matchedSrc->exportToOFF("matchedSrc" + std::to_string(i) + ".off");
    //   matchedDst->exportToOFF("matchedDst" + std::to_string(i) + ".off");
    // }

    VertexList::Vector sourcePoints;
    VertexList::Vector sourceNormals;
    VertexList::Vector targetPoints;
    VertexList::Vector targetNormals;

    // Add all matches to the sourcePoints and targetPoints vector,
    // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
    // the matches vector holds the index of the matching target point.
    for (size_t j = 0; j < trSource->vertices.size(); j++) {
      const auto& match = matches->matches[j];
      if (match.idx >= 0) {
        sourcePoints.push_back(trSource->vertices[j]);
        sourceNormals.push_back(trSource->normals[j]);

        targetPoints.push_back(dst_sampled->vertices[match.idx]);
        targetNormals.push_back(dst_sampled->normals[match.idx]);
      }
    }

    // Estimate the new pose
    if (this->error_metric == ErrorMetric::PointToPlane) {
      linear::PointToPlane p2plane(sourcePoints, targetPoints, targetNormals);
      estimatedPose = p2plane() * estimatedPose;
    } else if (this->error_metric == ErrorMetric::PointToPoint) {
      // estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
      linear::PointToPoint p2point(sourcePoints, targetPoints);
      estimatedPose = p2point() * estimatedPose;
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
  std::cout << std::endl << "Completed in " << elapsedSecs << " seconds." << std::endl;

  // Store result
  initialPose = estimatedPose;
}