#pragma once

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <memory>
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

    TrtCommon(std::string model_file_path);

    ~TrtCommon()  {} ;

    bool loadEngine(std::string engine_file_path);

    bool buildEngineFromOnnx(std::string onnx_file_path,
                             std::string output_engine_file_path);

    size_t numTensorElements(nvinfer1::Dims dimensions);

    void setup();


    template <typename T>
      using UniquePtr = std::unique_ptr<T, InferDeleter>;

    Logger g_logger;

    bool is_initialized;

    std::string model_file_path;

    UniquePtr<nvinfer1::IRuntime> runtime;

    UniquePtr<nvinfer1::ICudaEngine> engine;

    UniquePtr<nvinfer1::IExecutionContext> context;

    int input_binding_index;

    int output_binding_index;


    // nchw
    nvinfer1::Dims input_dims;

    nvinfer1::Dims output_dims;

    size_t num_input;

    size_t num_output;

    std::string input_name;

    std::string output_name;

    std::string precision;

    std::string cache_dir;

  private:

  };

}
