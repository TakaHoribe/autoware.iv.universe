#pragma once

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <memory>
#include <numeric>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <boost/filesystem.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cudnn.h>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Tn {

  class Logger : public nvinfer1::ILogger
  {
  public:

    void log(Severity severity, const char * msg) override
    {
      if (severity != Severity::kINFO)
        std::cout << msg << std::endl;
    }
  };

  struct InferDeleter
  {
    template <typename T>
    void operator()(T* obj) const {
      if (obj) {
        obj->destroy();
      }
    }
  };

  class TrtCommon {
  public:
    TrtCommon(std::string model_path, std::string cache_dir, std::string precision);
    ~TrtCommon()  {} ;

    bool loadEngine(std::string engine_file_path);
    bool buildEngineFromOnnx(std::string onnx_file_path,
                             std::string output_engine_file_path);
    void setup();

    bool isInitialized();
    int getNumInput();
    int getNumOutput();
    int getInputBindingIndex();
    int getOutputBindingIndex();

    template <typename T>
      using UniquePtr = std::unique_ptr<T, InferDeleter>;
    UniquePtr<nvinfer1::IExecutionContext> context_;

  private:
    Logger logger_;
    bool is_initialized_;
    size_t max_batch_size_;
    std::string model_file_path_;
    UniquePtr<nvinfer1::IRuntime> runtime_;
    UniquePtr<nvinfer1::ICudaEngine> engine_;

    nvinfer1::Dims input_dims_;
    nvinfer1::Dims output_dims_;
    std::string input_name_;
    std::string output_name_;
    std::string precision_;
    std::string cache_dir_;

  };

}
