#include "traffic_light_classifier/cnn_classifier.hpp"

namespace traffic_light {
  CNNClassifier::CNNClassifier(std::string label_file_path,
                               std::string model_file_path,
                               std::string precision,
                               std::string input_binding_name,
                               std::string output_binding_name)
    : nh_(""), pnh_("~"), image_transport_(pnh_) {
    image_pub_ = image_transport_.advertise("output/debug/image", 1);

    label_file_path_ = label_file_path;
    model_file_path_ = model_file_path;
    precision_ = precision;
    input_name_ = input_binding_name;
    output_name_ = output_binding_name;

    setup();
  }

  bool CNNClassifier::setup() {
    readLabelfile(label_file_path_, labels_);

    const boost::filesystem::path path(model_file_path_);
    std::string extension = path.extension().string();

    std::cout << "extension: " << extension << std::endl;

    if ( boost::filesystem::exists(path) ) {
      if ( extension == ".engine" ) {
        ROS_INFO("find engine file [%s]", model_file_path_.c_str());
        loadEngine(model_file_path_);
      } else if ( extension == ".onnx" ) {

        std::string cache_dir = ros::package::getPath("traffic_light_classifier") + "/data";
        std::string cache_engine_path = cache_dir + "/" + path.stem().string() + ".engine";
        const boost::filesystem::path cache_path(cache_engine_path);
        if ( boost::filesystem::exists(cache_path) ) {
          ROS_INFO("load engine from cache file");
          loadEngine(cache_engine_path);
        } else {
          ROS_INFO("build engine from onnx file");
          buildEngineFromOnnx(model_file_path_, cache_engine_path);
          ROS_INFO("complete build engine");
        }
      } else {
        ROS_WARN("Does not support this file extension [%s], \nplease input .engine or .onnx file",
                 extension.c_str());
      }
    } else {
      ROS_ERROR("Could not find file [%s]", model_file_path_.c_str());
    }
 
    ROS_INFO("create execution context");
    context_ = UniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());

    input_binding_index_ = engine_->getBindingIndex(input_name_.c_str());
    output_binding_index_ = engine_->getBindingIndex(output_name_.c_str());
    input_dims_ = engine_->getBindingDimensions(input_binding_index_);
    output_dims_ = engine_->getBindingDimensions(output_binding_index_);
    num_input_ = numTensorElements(input_dims_);
    num_output_ = numTensorElements(output_dims_);

    return true;
  }

  bool CNNClassifier::loadEngine(std::string engine_file_path) {
    std::ifstream engine_file(engine_file_path);
    std::stringstream engine_buffer;
    engine_buffer << engine_file.rdbuf();
    std::string engine_str = engine_buffer.str();
    runtime_ = UniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(g_logger_));
    engine_ = UniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine
                                               ((void*)engine_str.data(),
                                                engine_str.size(),
                                                nullptr));
    return true;
  }

  bool CNNClassifier::buildEngineFromOnnx(std::string onnx_file_path,
                                          std::string output_engine_file_path) {
    auto builder = UniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(g_logger_));
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = UniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
    auto config = UniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());

    auto parser = UniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, g_logger_));
    if ( !parser->parseFromFile(onnx_file_path.c_str(),
                                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)) ) {
      return false;
    }
    
    size_t max_batch_size = 1;
    builder->setMaxBatchSize(max_batch_size);
    builder->setMaxWorkspaceSize(16 << 20);


    if ( precision_ == "fp16" ) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    } else if ( precision_ == "int8" ) {
      config->setFlag(nvinfer1::BuilderFlag::kINT8);
    } else {
      ROS_WARN("Does not support this precision type [%s] \nplease input fp16 or int8", precision_.c_str());
    }

    engine_ = UniquePtr<nvinfer1::ICudaEngine>(builder->buildEngineWithConfig(*network, *config));
    if ( !engine_ ) {
      ROS_ERROR("failed to build engine");
      return false;
    }

    // save engine
    nvinfer1::IHostMemory *data = engine_->serialize();
    std::ofstream file;
    file.open(output_engine_file_path, std::ios::binary | std::ios::out);
    if (!file.is_open()) {
      std::cerr << "failed to output engine file" << std::endl;
    }
    file.write((const char*)data->data(), data->size());
    file.close();

    return true;
  }

  bool CNNClassifier::getLampState(const cv::Mat& input_image,
                                   std::vector<autoware_traffic_light_msgs::LampState>& states) {
    float *input_data_host = (float*) malloc(num_input_ * sizeof(float));

    cv::Mat image = input_image.clone();
    preProcess(image, input_data_host, true);

    float *input_data_device;
    cudaMalloc((void**)&input_data_device, num_input_ * sizeof(float));
    cudaMemcpy(input_data_device,
               input_data_host,
               num_input_ * sizeof(float),
               cudaMemcpyHostToDevice);

    float *output_data_device;
    cudaMalloc((void**)&output_data_device,
               num_output_ * sizeof(float));

    // do inference
    void *bindings[2];
    bindings[input_binding_index_] = (void*)input_data_device;
    bindings[output_binding_index_] = (void*)output_data_device;

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    context_->executeV2(bindings);
    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    double elapsed_time = static_cast<double>
      (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) / 1000;
    // ROS_INFO("inference elapsed time: %f [ms]", elapsed_time);

    float *output_data_host = (float*) malloc(num_output_ * sizeof(float));
    cudaMemcpy(output_data_host,
               output_data_device,
               num_output_ * sizeof(float),
               cudaMemcpyDeviceToHost);

    postProcess(output_data_host, states);

    /* debug */
    if (0 < image_pub_.getNumSubscribers()) {
      float probability;
      std::string label;
      for (auto state : states) {
        // all lamp confidence are the same
        probability = state.confidence;
        label += state2label_[state.type];
      }

      cv::Mat debug_image = input_image.clone();
      int expand_w = 200;
      int expand_h = static_cast<int>((expand_w * input_image.rows) / input_image.cols);

      cv::resize(input_image, debug_image, cv::Size(expand_w, expand_h));

      cv::Mat text_img(cv::Size(expand_w, 50), CV_8UC3, cv::Scalar(0,0,0));
      std::string text = label + " " + std::to_string(probability);

      cv::putText(text_img,
                  text,
                  cv::Point(5, 25),
                  cv::FONT_HERSHEY_COMPLEX,
                  0.5,
                  cv::Scalar(0,255,0), 1);

      cv::Mat concated_image;
      cv::vconcat(debug_image, text_img, concated_image);

      sensor_msgs::ImagePtr debug_image_msg = cv_bridge::CvImage
        (std_msgs::Header(), "bgr8", concated_image).toImageMsg();
      image_pub_.publish(debug_image_msg);
    }

    cudaFree(input_data_device);
    cudaFree(output_data_device);

    return true;
  }

  void CNNClassifier::preProcess(cv::Mat & image,
                                 float *tensor,
                                 bool normalize)
  {
    /* normalize */
    /* ((channel[0] / 255) - mean[0]) / std[0] */

    cv::cvtColor(image, image, cv::COLOR_BGR2RGB, 3);
    cv::resize(image, image, cv::Size(input_dims_.d[2], input_dims_.d[3]));

    const size_t channels = input_dims_.d[1];
    const size_t height = input_dims_.d[2];
    const size_t width = input_dims_.d[3];

    const size_t stridesCv[3] = { width * channels, channels, 1 };
    const size_t strides[3] = { height * width, width, 1 };

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        for (int k = 0; k < channels; k++) {
          const size_t offsetCv = i * stridesCv[0] + j * stridesCv[1] + k * stridesCv[2];
          const size_t offset = k * strides[0] + i * strides[1] + j * strides[2];
          if (normalize) {
            tensor[offset] = (((float) image.data[offsetCv] / 255) - mean_[k]) / std_[k];
          } else {
            tensor[offset] = (float) image.data[offsetCv];
          }
        }
      }
    }

  }

  bool CNNClassifier::postProcess(float* output_data_host,
                                  std::vector<autoware_traffic_light_msgs::LampState>& states)
  {
    std::vector<float> probs;
    calcSoftmax(output_data_host, probs);
    std::vector<size_t> sorted_indices = argsort(output_data_host);

    // ROS_INFO("label: %s, score: %.2f\%",
    //          labels_[sorted_indices[0]].c_str(),
    //          probs[sorted_indices[0]] * 100);

    // TODO
    // correspond arrow label and lamp state
    // 

    std::string match_label = labels_[sorted_indices[0]];
    float probability = probs[sorted_indices[0]];
    if (match_label == "stop") {
      autoware_traffic_light_msgs::LampState state;
      state.type = autoware_traffic_light_msgs::LampState::RED;
      state.confidence = probability;
      states.push_back(state);
    } else if (match_label == "warning") {
      autoware_traffic_light_msgs::LampState state;
      state.type = autoware_traffic_light_msgs::LampState::YELLOW;
      state.confidence = probability;
      states.push_back(state);
    } else if (match_label == "go") {
      autoware_traffic_light_msgs::LampState state;
      state.type = autoware_traffic_light_msgs::LampState::GREEN;
      state.confidence = probability;
      states.push_back(state);
    } else {
      autoware_traffic_light_msgs::LampState state;
      state.type = autoware_traffic_light_msgs::LampState::UNKNOWN;
      state.confidence = 0.0;
      states.push_back(state);
    }

    return true;
  }


  bool CNNClassifier::readLabelfile(std::string filepath,
                                    std::vector<std::string>& labels)
  {
    std::ifstream labelsFile(filepath);
    if (!labelsFile.is_open()) {
      ROS_ERROR("Could not open label file. [%s]", filepath.c_str());
      return false;
    }
    std::string label;
    while(getline(labelsFile, label)) {
      labels.push_back(label);
    }
    return true;
  }

  void CNNClassifier::calcSoftmax(float *data,
                                  std::vector<float>& probs)
  {
    float exp_sum = 0.0;
    for (int i=0; i<num_output_; ++i) {
      exp_sum += exp(data[i]);
    }

    for (int i=0; i<num_output_; ++i) {
      probs.push_back(exp(data[i]) / exp_sum);
    }
  }

  std::vector<size_t> CNNClassifier::argsort(float *tensor)
  {
    std::vector<size_t> indices(num_output_);
    for (int i = 0; i < num_output_; i++)
      indices[i] = i;
    std::sort(indices.begin(), indices.begin() + num_output_, [tensor](size_t idx1, size_t idx2) {
        return tensor[idx1] > tensor[idx2];
      });

    return indices;
  }

  size_t CNNClassifier::numTensorElements(nvinfer1::Dims dimensions)
  {
    if (dimensions.nbDims == 0)
      return 0;
    size_t size = 1;
    for (int i = 0; i < dimensions.nbDims; i++)
      size *= dimensions.d[i];
    return size;
  }

}  // namespace traffic_light
