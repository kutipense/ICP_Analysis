#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__
#include <data_types/MatchList.h>
#include <data_types/VertexList.h>
#include <discard/Discard.h>
#include <matching/Matcher.h>
#include <sampling/Sampler.h>

#include <Eigen/Dense>
#include <memory>

enum class ErrorMetric { PointToPoint, PointToPlane, Symmetric };

template <typename T, typename M, typename SamplerType, typename DiscardType, typename MatcherType,
          typename std::enable_if<std::is_same<VertexList, T>::value, T>::type* = nullptr,
          typename std::enable_if<std::is_same<MatchList, M>::value, M>::type*  = nullptr>
class Optimizer {
 public:
  Optimizer(typename T::Ptr& source, typename T::Ptr& target, ErrorMetric error_metric = ErrorMetric::PointToPoint,
            unsigned int m_nIterations = 20)
      : source{source}, target{target}, error_metric{error_metric}, m_nIterations{m_nIterations} {}
  virtual void optimize(Eigen::Matrix4f& initialPose) = 0;
  void         setNumOfIterations(unsigned int nIterations) { m_nIterations = nIterations; }
  virtual ~Optimizer() = default;

 protected:
  unsigned int                   m_nIterations;
  std::shared_ptr<Discard<T, M>> discarder;
  std::shared_ptr<Matcher<M>>    matcher;
  std::shared_ptr<Sampler<T>>    sampler;
  ErrorMetric                    error_metric;
  typename T::Ptr&               source;
  typename T::Ptr&               target;

  std::vector<Eigen::Vector3f> transformPoints(const VertexList::Vector& sourcePoints, const Matrix4f& pose) {
    std::vector<Eigen::Vector3f> transformedPoints;
    transformedPoints.reserve(sourcePoints.size());

    const auto rotation    = pose.block(0, 0, 3, 3);
    const auto translation = pose.block(0, 3, 3, 1);

    for (const auto& point : sourcePoints) {
      Eigen::Vector3f _point(point[0], point[1], point[2]);
      auto const      v = rotation * _point + translation;
      transformedPoints.emplace_back(v);
    }

    return transformedPoints;
  }

  void prepare() {
    typename SamplerType::Ptr sampler = std::make_shared<SamplerType>(source);

    VertexList::Ptr out = sampler.sample();
  }

  std::vector<Eigen::Vector3f> transformNormals(const VertexList::Vector& sourceNormals, const Eigen::Matrix4f& pose) {
    std::vector<Eigen::Vector3f> transformedNormals;
    transformedNormals.reserve(sourceNormals.size());

    const auto rotation = pose.block(0, 0, 3, 3);
    for (const auto& normal : sourceNormals) {
      Eigen::Vector3f _normal(normal[0], normal[1], normal[2]);
      auto const      v = rotation.inverse().transpose() * _normal;
      transformedNormals.emplace_back(v);
    }

    return transformedNormals;
  }
};

#endif