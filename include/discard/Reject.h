#ifndef __REJECT_H__
#define __REJECT_H__

#include <discard/Discard.h>

#include <optional>

namespace discard {
class Reject : public Discard {
 public:
  using Ptr           = std::shared_ptr<Reject>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  void setMaxDistance(double distance);
  void setMaxAngle(double angle);

  MatchPtr discard(DataPtr source_data, DataPtr target_data, MatchPtr match_list) override;

 private:
  std::optional<double> dist_;
  std::optional<double> ang_;
};
}  // namespace discard
#endif