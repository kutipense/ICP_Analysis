#ifndef __SURFACE_NORMAL_REJECT_H__
#define __SURFACE_NORMAL_REJECT_H__

#include <discard/Discard.h>

#include <optional>

namespace discard {
class SurfaceNormalReject : public Discard {
 public:
  using Ptr           = std::shared_ptr<SurfaceNormalReject>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  SurfaceNormalReject(DataPtr source_data, DataPtr target_data, MatchPtr match_list);

  SurfaceNormalReject(const SurfaceNormalReject&) = delete;
  SurfaceNormalReject operator=(const SurfaceNormalReject&) = delete;

  void setMaxAngle(double angle);

  MatchPtr discard() override;

 private:
  std::optional<double> dist_;
  std::optional<double> ang_;
};
}  // namespace discard
#endif