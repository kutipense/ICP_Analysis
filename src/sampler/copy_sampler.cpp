#include <sampler/CopySampler.h>

namespace sampler {
VertexList::Ptr CopySampler::sample() {
  VertexList::Ptr data_copy = std::make_shared<VertexList>();

  *data_copy = *(this->data_);

  return data_copy;
}
}  // namespace sampler