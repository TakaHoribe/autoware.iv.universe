#include "traffic_light_classifier/cnn_classifier.hpp"

namespace traffic_light {
  CNNClassifier::CNNClassifier() : nh_(""), pnh_("~"), image_transport_(pnh_) {
    pnh_.param<std::string>("engine_file_path", engine_file_path_);
    pnh_.param<std::string>("label_file_path", label_file_path_);

    image_pub_ = image_transport_.advertise("output/debug/image", 1);

    readLabelfile(label_file_path_, labels_);

    /* load the engine */
    std::ifstream engine_file(engine_file_path_);
    if (!engine_file.is_open()) {
      ROS_ERROR("Could not open engine file.");
    }
    std::stringstream engine_buffer;
    engine_buffer << engine_file.rdbuf();
    std::string engine_str = engine_buffer.str();

    runtime_ = UniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(g_logger_));
    engine_ = UniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine
                                               ((void*)engine_str.data(),
                                                engine_str.size(),
                                                nullptr));
    context_ = UniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());

    input_binding_index_ = engine_->getBindingIndex(input_name_.c_str());
    output_binding_index_ = engine_->getBindingIndex(output_name_.c_str());
    input_dims_ = engine_->getBindingDimensions(input_binding_index_);
    output_dims_ = engine_->getBindingDimensions(output_binding_index_);
    num_input_ = numTensorElements(input_dims_);
    num_output_ = numTensorElements(output_dims_);
  }

  bool CNNClassifier::getLampState(const cv::Mat& input_image,
                                   std::vector<autoware_traffic_light_msgs::LampState>& states) {
    float *input_data_host = (float*) malloc(num_input_ * sizeof(float));
    preProcess(input_image, input_data_host, true);

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
    context_->executeV2(bindings);

    float *output_data_host = (float*) malloc(num_output_ * sizeof(float));
    cudaMemcpy(output_data_host,
               output_data_device,
               num_output_ * sizeof(float),
               cudaMemcpyDeviceToHost);

    postProcess(output_data_host);

    cudaFree(input_data_device);
    cudaFree(output_data_device);

    return true;
  }

  void CNNClassifier::preProcess(const cv::Mat & image,
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

  bool CNNClassifier::postProcess(float* output_data_host)
  {
    std::vector<float> probs;
    calcSoftmax(output_data_host, probs);
    std::vector<size_t> sorted_indices = argsort(output_data_host);
    // ROS_INFO("label: %s, score: %.2f\%\n",
    //          labels[sortedIndices[0]].c_str(),
    //          probs[sortedIndices[0]] * 100);

    return true;
  }


  bool CNNClassifier::readLabelfile(std::string filepath,
                                    std::vector<std::string>& labels)
  {
    std::ifstream labelsFile(filepath);
    if (!labelsFile.is_open()) {
      std::cout << "\nCould not open label file." << std::endl;
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
