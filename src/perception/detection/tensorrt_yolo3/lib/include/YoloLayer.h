#ifndef _YOLO_LAYER_H
#define _YOLO_LAYER_H

#include <assert.h>
#include <cublas_v2.h>
#include <cudnn.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include "NvInfer.h"
#include "Utils.h"

namespace Yolo {
struct YoloKernel;

static constexpr int LOCATIONS = 4;
struct Detection {
  // x y w h
  float bbox[LOCATIONS];
  // float objectness;
  int classId;
  float prob;
};
}  // namespace Yolo

namespace nvinfer1 {
class YoloLayerPlugin : public IPluginExt {
 public:
  explicit YoloLayerPlugin(const int cudaThread = 512);
  YoloLayerPlugin(const void* data, size_t length);

  ~YoloLayerPlugin();

  int getNbOutputs() const override { return 1; }

  Dims getOutputDimensions(int index, const Dims* inputs, int nbInputDims) override;

  bool supportsFormat(DataType type, PluginFormat format) const override {
    return type == DataType::kFLOAT && format == PluginFormat::kNCHW;
  }

  void configureWithFormat(const Dims* inputDims, int nbInputs, const Dims* outputDims, int nbOutputs, DataType type,
                           PluginFormat format, int maxBatchSize) override{};

  int initialize() override;

  virtual void terminate() override{};

  virtual size_t getWorkspaceSize(int maxBatchSize) const override { return 0; }

  virtual int enqueue(int batchSize, const void* const* inputs, void** outputs, void* workspace,
                      cudaStream_t stream) override;

  virtual size_t getSerializationSize() override;

  virtual void serialize(void* buffer) override;

  void forwardGpu(const float* const* inputs, float* output, cudaStream_t stream);

  void forwardCpu(const float* const* inputs, float* output, cudaStream_t stream);

 private:
  int mClassCount;
  int mKernelCount;
  std::vector<Yolo::YoloKernel> mYoloKernel;
  int mThreadCount;

  // cpu
  void* mInputBuffer{nullptr};
  void* mOutputBuffer{nullptr};
};
};  // namespace nvinfer1

#endif
