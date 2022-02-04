#include <discard/SurfaceNormalReject.h>

namespace discard {

SurfaceNormalReject::SurfaceNormalReject(DataPtr source_data, DataPtr target_data, MatchPtr match_list)
    : Discard(source_data, target_data, match_list) {}

void SurfaceNormalReject::setMaxAngle(double angle) { ang_ = angle; }

SurfaceNormalReject::MatchPtr SurfaceNormalReject::discard() {
  // auto inputPc      = DataType::toPCL<pcl::PointXYZ>(this->source_data->vertices);
  // auto targetPc     = DataType::toPCL<pcl::PointXYZ>(this->target_data_->vertices);
  // auto inputNormals = DataType::toPCL<pcl::PointXYZ>(this->target_data_->vertices);

  // pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej(
  //     new pcl::registration::CorrespondenceRejectorSurfaceNormal);

  // rej->setInputSource(this->source_data_->vertices);
  // rej->setInputNormals(this->source_data_->normals);
  // rej->setInputTarget(this->target_data_->vertices);
  // rej->setTargetNormals(this->target_data_->normals);
  // rej->setThreshold(ang_ != nullptr ? ang_.value() : 0);
  // rej->setInputCorrespondences(remaining_correspondences_temp);
  // rej->getCorrespondences(remaining_correspondences);

  // pcl::Correspondences all_correspondences;
  // rej->determineCorrespondences(all_correspondences);

  // MatchPtr matches_pruned = std::make_shared<MatchType>();
  // matches_pruned->matches.reserve(all_correspondences.size());
  // for (size_t i = 0; i < all_correspondences.size(); i++)
  //   matches_pruned->matches.push_back({all_correspondences[i].index_match, 1.0});

  // return matches_pruned;
  return std::make_shared<MatchType>();
}

}  // namespace discard