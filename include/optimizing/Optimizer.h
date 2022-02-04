#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__
#include <data_types/MatchList.h>
#include <data_types/VertexList.h>
#include <discard/Discard.h>
#include <matcher/Matcher.h>
#include <sampler/Sampler.h>

#include <Eigen/Dense>
#include <memory>

enum class ErrorMetric { PointToPoint, PointToPlane, Symmetric };

template <typename SamplerType, typename DiscardType, typename MatcherType>
class Optimizer {
 public:
  using DataType = VertexList;
  using DataPtr  = VertexList::Ptr;

  Optimizer(DataPtr source, DataPtr target, ErrorMetric error_metric = ErrorMetric::PointToPoint,
            unsigned int m_nIterations = 20, const float weight = 2)
      : source{source}, target{target}, error_metric{error_metric}, m_nIterations{m_nIterations}, weight{weight} {}
  virtual void optimize(Eigen::Matrix4f& initialPose) = 0;
  void         setNumOfIterations(unsigned int nIterations) { m_nIterations = nIterations; }
  virtual ~Optimizer() = default;

 protected:
  unsigned int                      m_nIterations;
  float                             weight;
  std::shared_ptr<discard::Discard> discarder;
  std::shared_ptr<matcher::Matcher> matcher;
  std::shared_ptr<sampler::Sampler> sampler;
  ErrorMetric                       error_metric;
  DataPtr                           source;
  DataPtr                           target;

  DataType::Vector transformPoints(const DataType::Vector& sourcePoints, const Eigen::Matrix4f& pose) {
    return Eigen::transformPoints(sourcePoints, pose);
  }

  DataType::Vector transformNormals(const DataType::Vector& sourceNormals, const Eigen::Matrix4f& pose) {
    return Eigen::transformNormals(sourceNormals, pose);
  }
};

#endif