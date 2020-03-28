#include "trt_common.h"

namespace Tn {

  TrtCommon::TrtCommon(std::string model)
    : cache_dir(""), precision("fp16"),
      input_name("input_0"), output_name("output_0"),
      is_initialized(false) {
    model_file_path = model;
  }

  void TrtCommon::setup() {
    const boost::filesystem::path path(model_file_path);
    std::string extension = path.extension().string();

    if ( boost::filesystem::exists(path) ) {
      if ( extension == ".engine" ) {
        loadEngine(model_file_path);
      } else if ( extension == ".onnx" ) {
        std::string cache_engine_path = cache_dir + "/" + path.stem().string() + ".engine";
        const boost::filesystem::path cache_path(cache_engine_path);
        if ( boost::filesystem::exists(cache_path) ) {
          loadEngine(cache_engine_path);
        } else {
          buildEngineFromOnnx(model_file_path, cache_engine_path);
        }
      } else {
        is_initialized = false;
      }
    } else {
      is_initialized = false;
    }

    context = UniquePtr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    input_binding_index = engine->getBindingIndex(input_name.c_str());
    output_binding_index = engine->getBindingIndex(output_name.c_str());
    input_dims = engine->getBindingDimensions(input_binding_index);
    output_dims = engine->getBindingDimensions(output_binding_index);
    num_input = numTensorElements(input_dims);
    num_output = numTensorElements(output_dims);

    is_initialized = true;
  }

  bool TrtCommon::loadEngine(std::string engine_file_path) {
    std::ifstream engine_file(engine_file_path);
    std::stringstream engine_buffer;
    engine_buffer << engine_file.rdbuf();
    std::string engine_str = engine_buffer.str();
    runtime = UniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(g_logger));
    engine = UniquePtr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine
                                              ((void*)engine_str.data(),
                                               engine_str.size(),
                                               nullptr));
    return true;
  }

  bool TrtCommon::buildEngineFromOnnx(std::string onnx_file_path,
                                      std::string output_engine_file_path) {
    auto builder = UniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(g_logger));
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = UniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
    auto config = UniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());

    auto parser = UniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, g_logger));
    if ( !parser->parseFromFile(onnx_file_path.c_str(),
                                static_cast<int>(nvinfer1::ILogger::Severity::kERROR)) ) {
      return false;
    }
    
    size_t max_batch_size = 1;
    builder->setMaxBatchSize(max_batch_size);
    builder->setMaxWorkspaceSize(16 << 20);

    if ( precision == "fp16" ) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else if ( precision == "int8" ) {
      config->setFlag(nvinfer1::BuilderFlag::kINT8);
    } else {
      return false;
    }

    engine = UniquePtr<nvinfer1::ICudaEngine>(builder->buildEngineWithConfig(*network, *config));
    if ( !engine )
      return false;

    // save engine
    nvinfer1::IHostMemory *data = engine->serialize();
    std::ofstream file;
    file.open(output_engine_file_path, std::ios::binary | std::ios::out);
    if (!file.is_open())
      return false;
    file.write((const char*)data->data(), data->size());
    file.close();

    return true;
  }

  size_t TrtCommon::numTensorElements(nvinfer1::Dims dimensions)
  {
    if (dimensions.nbDims == 0)
      return 0;
    size_t size = 1;
    for (int i = 0; i < dimensions.nbDims; i++)
      size *= dimensions.d[i];
    return size;
  }

} // Tn
