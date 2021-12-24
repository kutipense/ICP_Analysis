#ifndef __PROJECTIVE_MATCHER_H__
#define __PROJECTIVE_MATCHER_H__

#include <matching/Matcher.h>

#include <flann/flann.hpp>

template <typename T>
class ProjectiveMatcher : public Matcher<T> {
 public:
  using Ptr           = std::shared_ptr<ProjectiveMatcher<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  ProjectiveMatcher(typename T::Ptr source_data_ptr, typename T::Ptr target_data_ptr, int number_of_checks = 16,
                    float max_distance = 1, int tree_size = 1)
      : Matcher<T>(source_data_ptr, target_data_ptr),
        num_of_checks_(number_of_checks),
        max_distance_(max_distance),
        tree_size_(tree_size) {}

  ProjectiveMatcher(const ProjectiveMatcher&) = delete;
  ProjectiveMatcher operator=(const ProjectiveMatcher&) = delete;

  MatchList::Ptr match() override {
    // std::vector<Eigen::Vector3d> prev_frame_global_points = prev_frame->getGlobalPoints();
    // std::vector<Eigen::Vector3d> prev_frame_global_normals = prev_frame->getGlobalNormals();

    // std::vector<Eigen::Vector3d> curr_frame_points = curr_frame->getPoints();
    // std::vector<Eigen::Vector3d> curr_frame_normals = curr_frame->getNormals();

    // const auto rotation = estimated_pose.block(0, 0, 3, 3);
    // const auto translation = estimated_pose.block(0, 3, 3, 1);

    // for(size_t idx = 0; idx < curr_frame_points.size(); idx++){

    //     Eigen::Vector3d curr_point = curr_frame_points[idx];
    //     Eigen::Vector3d curr_normal = curr_frame_normals[idx];

    //     if (curr_point.allFinite() && curr_normal.allFinite()) {
    //         const Eigen::Vector3d curr_global_point = rotation * curr_point + translation;
    //         const Eigen::Vector3d curr_global_normal = rotation * curr_normal;

    //         const Eigen::Vector3d curr_point_prev_frame = prev_frame->projectIntoCamera(curr_global_point);
    //         const Eigen::Vector2i curr_point_img_coord = prev_frame->projectOntoDepthPlane(curr_point_prev_frame);

    //         if (prev_frame->contains(curr_point_img_coord)) {

    //             size_t prev_idx = curr_point_img_coord[1] * prev_frame->getWidth() + curr_point_img_coord[0];

    //             Eigen::Vector3d prev_global_point = prev_frame_global_points[prev_idx];
    //             Eigen::Vector3d prev_global_normal = prev_frame_global_normals[prev_idx];

    //             if (prev_global_point.allFinite() && prev_global_normal.allFinite()) {

    //                 if(hasValidDistance(prev_global_point, curr_global_point) &&
    //                    hasValidAngle(prev_global_normal, curr_global_normal)) {

    //                     corresponding_points.push_back(std::make_pair(prev_idx, idx));
    //                 }
    //             }
    //         }
    //     }
    // }
  }

 private:
  int   num_of_checks_, tree_size_;
  float max_distance_;
};

#endif