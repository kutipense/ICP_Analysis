#ifndef __REJECT_H__
#define __REJECT_H__

#include <discard/Discard.h>

#include <optional>

namespace discard {
class Reject : public Discard {
 public:
  using Ptr           = std::shared_ptr<Reject>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  Reject(Discard::DataPtr source_data, Discard::DataPtr target_data, MatchPtr match_list);

  Reject(const Reject&) = delete;
  Reject operator=(const Reject&) = delete;

  void setMaxDistance(double distance);
  void setMaxAngle(double angle);

  MatchPtr discard() override;

 private:
  std::optional<double> dist_;
  std::optional<double> ang_;
};
}  // namespace discard
#endif