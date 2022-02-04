#include <sampler/CopySampler.h>

namespace sampler {
CopySampler::CopySampler(DataPtr data_ptr) : Sampler(data_ptr) {}

CopySampler::DataPtr CopySampler::sample() {
  DataPtr data_copy = std::make_shared<DataType>();

  *data_copy = *(this->data_);

  return data_copy;
}
}  // namespace sampler