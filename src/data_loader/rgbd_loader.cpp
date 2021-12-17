#include <data_loader/RgbdLoader.h>

#include <memory>

// utilty
#define IS_INVALID(X) ((X) == MINF)
#define IS_VALID(X) ((X) != MINF)
#define IS_VALID_P(X) (IS_VALID((X).x()))
#define IS_INVALID_P(X) (IS_INVALID((X).x()))

RgbdLoader::RgbdLoader(const std::string& data_src) { sensor_.Init(data_src); }

VertexList::Ptr RgbdLoader::load() {
  if (!sensor_.ProcessNextFrame()) return nullptr;

  auto rgbdData = std::make_shared<VertexList>();

  float* depthMap = sensor_.GetDepth();
  BYTE*  colorMap = sensor_.GetColorRGBX();

  Matrix3f depthIntrinsics    = sensor_.GetDepthIntrinsics();
  Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

  float fX = depthIntrinsics(0, 0), fY = depthIntrinsics(1, 1);
  float cX = depthIntrinsics(0, 2), cY = depthIntrinsics(1, 2);

  Matrix4f depthExtrinsicsInv = sensor_.GetDepthExtrinsics().inverse();

  Matrix4f trajectory    = sensor_.GetTrajectory();
  Matrix4f trajectoryInv = sensor_.GetTrajectory().inverse();

  size_t size = sensor_.GetDepthImageWidth() * sensor_.GetDepthImageHeight();
  rgbdData->vertices.reserve(size);
  rgbdData->colors.reserve(size);

  int dIndex = 0;  // linear index counter
  for (int y = 0; y < sensor_.GetDepthImageHeight(); y++) {
    for (int x = 0; x < sensor_.GetDepthImageWidth(); x++) {
      float z = depthMap[dIndex];
      if (IS_VALID(z)) {
        Eigen::Vector3f wVector;
        Eigen::Vector4f position;

        wVector << x * z, y * z, z;
        wVector = depthIntrinsicsInv * wVector;
        position << wVector, 1;
        position = trajectoryInv * position;
        rgbdData->vertices.push_back({position(0), position(1), position(2)});

        auto cIndex = dIndex * 4;  // color and depth are aligned
        rgbdData->colors.push_back(
            {colorMap[cIndex], colorMap[cIndex + 1], colorMap[cIndex + 2], colorMap[cIndex + 3]});
      } else {
        rgbdData->vertices.push_back({MINF, MINF, MINF});
        rgbdData->colors.push_back({0, 0, 0, 0});
      }
      dIndex++;  // increase linear counter
    }
  }

  return rgbdData;
}