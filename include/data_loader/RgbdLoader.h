#ifndef __RGBD_LOADER_H__
#define __RGBD_LOADER_H__

#include <data_loader/VertexLoader.h>
#include <external/VirtualSensor.h>

#include <array>
#include <vector>

class RgbdLoader : VertexLoader {
 public:
  RgbdLoader(const std::string&);
  RgbdLoader(const RgbdLoader&) = delete;
  RgbdLoader operator=(const RgbdLoader&) = delete;
  ~RgbdLoader()                           = default;

  VertexList::Ptr load() override;

 private:
  VirtualSensor sensor_;
};

#endif