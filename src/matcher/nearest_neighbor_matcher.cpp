#include <matcher/NearestNeighborMatcher.h>

#include <flann/flann.hpp>

namespace matcher {
NearestNeighborMatcher::NearestNeighborMatcher(int number_of_checks, float max_distance, int tree_size)
    : num_of_checks_(number_of_checks), max_distance_(max_distance), tree_size_(tree_size) {}

MatchList::Ptr NearestNeighborMatcher::match(const VertexList::Vector& source_data,
                                             const VertexList::Vector& target_data) {
  auto matchPtr = std::make_shared<MatchList>();

  // FLANN requires that all the points be flat. Therefore we copy the points to a separate flat array.
  float* m_flatPoints = new float[target_data.size() * 3];
  for (size_t pointIndex = 0; pointIndex < target_data.size(); pointIndex++) {
    for (size_t dim = 0; dim < 3; dim++) { m_flatPoints[pointIndex * 3 + dim] = target_data[pointIndex][dim]; }
  }

  flann::Matrix<float> dataset(m_flatPoints, target_data.size(), 3);

  // Building the index takes some time.
  flann::Index<flann::L2<float>>* m_index =
      new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(tree_size_));
  m_index->buildIndex();

  float* queryPoints = new float[source_data.size() * 3];
  for (size_t pointIndex = 0; pointIndex < source_data.size(); pointIndex++) {
    for (size_t dim = 0; dim < 3; dim++) { queryPoints[pointIndex * 3 + dim] = source_data[pointIndex][dim]; }
  }

  flann::Matrix<float>  query(queryPoints, source_data.size(), 3);
  flann::Matrix<size_t> indices(new size_t[query.rows * 1], query.rows, 1);
  flann::Matrix<float>  distances(new float[query.rows * 1], query.rows, 1);

  flann::SearchParams searchParams{num_of_checks_};
  searchParams.cores = 0;
  m_index->knnSearch(query, indices, distances, 1, searchParams);

  // Filter the matches.
  const unsigned nMatches = source_data.size();
  matchPtr->matches.reserve(nMatches);

  for (int i = 0; i < nMatches; ++i) {
    if (*distances[i] <= max_distance_)
      matchPtr->matches.push_back(Match{(long long int)*indices[i], 1.f});
    else
      matchPtr->matches.push_back(Match{-1, 0.f});
  }

  delete[] query.ptr();
  delete[] indices.ptr();
  delete[] distances.ptr();
  delete[] dataset.ptr();
  delete m_index;
  return matchPtr;
}

}  // namespace matcher