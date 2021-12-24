#ifndef __NEAREST_NEIGHBOR_MATCHER_H__
#define __NEAREST_NEIGHBOR_MATCHER_H__

#include <flann/flann.hpp>
#include <matching/Matcher.h>

template <typename T>
class NearestNeighborMatcher : public Matcher<T> {
 public:
  using Ptr           = std::shared_ptr<NearestNeighborMatcher<T>>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  NearestNeighborMatcher(typename VertexList::Ptr source_data_ptr, typename VertexList::Ptr target_data_ptr, int number_of_checks=16, float max_distance=1, int tree_size=1) : 
    Matcher<T>(source_data_ptr, target_data_ptr), 
    num_of_checks_(number_of_checks),
    max_distance_(max_distance),
    tree_size_(tree_size) {}

  NearestNeighborMatcher(const NearestNeighborMatcher&) = delete;
  NearestNeighborMatcher operator=(const NearestNeighborMatcher&) = delete;

  typename T::Ptr match() override {
    auto matchPtr  = std::make_shared<MatchList>();

    // FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
    float* m_flatPoints = new float[this->source_data_->vertices.size() * 3];
    for (size_t pointIndex = 0; pointIndex < this->source_data_->vertices.size(); pointIndex++) {
      for (size_t dim = 0; dim < 3; dim++) {
        m_flatPoints[pointIndex * 3 + dim] = this->source_data_->vertices[pointIndex][dim];
      }
    }


    flann::Matrix<float> dataset(m_flatPoints, this->source_data_->vertices.size(), 3);

    // Building the index takes some time.
    flann::Index<flann::L2<float>>* m_index = new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(tree_size_));
    m_index->buildIndex();

    float* queryPoints = new float[this->target_data_->vertices.size() * 3];
    for (size_t pointIndex = 0; pointIndex < this->target_data_->vertices.size(); pointIndex++) {
      for (size_t dim = 0; dim < 3; dim++) {
        queryPoints[pointIndex * 3 + dim] = this->target_data_->vertices[pointIndex][dim];
      }
    }

    flann::Matrix<float> query(queryPoints, this->target_data_->vertices.size(), 3);
    flann::Matrix<int> indices(new int[query.rows * 1], query.rows, 1);
    flann::Matrix<float> distances(new float[query.rows * 1], query.rows, 1);

    flann::SearchParams searchParams{ num_of_checks_ };
    searchParams.cores = 0;
    m_index->knnSearch(query, indices, distances, 1, searchParams);

    // Filter the matches.
    const unsigned nMatches = this->target_data_->vertices.size();
    matchPtr->matches.reserve(nMatches);

    for (int i = 0; i < nMatches; ++i) {
      if (*distances[i] <= max_distance_)
        matchPtr->matches.push_back(Match{ *indices[i], 1.f });
      else
        matchPtr->matches.push_back(Match{ -1, 0.f });
    }

    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] distances.ptr();
    delete[] dataset.ptr();
    delete m_index;
    return matchPtr;
  }
  private:
    int num_of_checks_, tree_size_;
    float max_distance_;
};

#endif