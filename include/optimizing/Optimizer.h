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

class Optimizer {
 public:
  using DataType = VertexList;
  using DataPtr  = VertexList::Ptr;

  Optimizer(DataPtr source, DataPtr target)
      : source{source}, target{target}, error_metric{ErrorMetric::PointToPoint}, m_nIterations{20}, weight{2} {}

  virtual ~Optimizer() = default;

  virtual void optimize(Eigen::Matrix4f& initialPose) = 0;

  void setNumOfIterations(unsigned int nIterations) { m_nIterations = nIterations; }
  void setWeight(float _weight) { weight = _weight; }

  void setDiscarder(std::shared_ptr<discard::Discard> _discarder) { discarder = _discarder; }
  void setSrcSampler(std::shared_ptr<sampler::Sampler> _sampler) { src_sampler = _sampler; }
  void setDstSampler(std::shared_ptr<sampler::Sampler> _sampler) { dst_sampler = _sampler; }
  void setMatcher(std::shared_ptr<matcher::Matcher> _matcher) { matcher = _matcher; }

  void setErrorMetric(ErrorMetric _error_metric) { error_metric = _error_metric; }

 protected:
  unsigned int m_nIterations;
  float        weight;

  std::shared_ptr<discard::Discard> discarder;
  std::shared_ptr<matcher::Matcher> matcher;
  std::shared_ptr<sampler::Sampler> src_sampler, dst_sampler;

  ErrorMetric error_metric;
  DataPtr     source;
  DataPtr     target;

  DataType::Vector transformPoints(const DataType::Vector& sourcePoints, const Eigen::Matrix4f& pose) {
    return Eigen::transformPoints(sourcePoints, pose);
  }

  DataType::Vector transformNormals(const DataType::Vector& sourceNormals, const Eigen::Matrix4f& pose) {
    return Eigen::transformNormals(sourceNormals, pose);
  }
};

#endif