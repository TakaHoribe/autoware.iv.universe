#pragma once

#include <autoware_traffic_light_msgs/LampState.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <traffic_light_classifier/HSVFilterConfig.h>
#include <traffic_light_classifier/classifier_interface.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <algorithm>
#include <chrono>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace traffic_light {

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

  class CNNClassifier : public ClassifierInterface {
  public:
    CNNClassifier();

    ~CNNClassifier(){};

    bool getLampState(const cv::Mat& input_image, std::vector<autoware_traffic_light_msgs::LampState>& states) override;

  private:

    void parametersCallback(traffic_light_classifier::HSVFilterConfig& config, uint32_t level);

    void preProcess(const cv::Mat & image, float *tensor, bool normalize=true);

    bool postProcess(float* output_data_host);

    bool readLabelfile(std::string filepath, std::vector<std::string>& labels);

    void calcSoftmax(float *data, std::vector<float>& probs);

    std::vector<size_t> argsort(float *tensor);

    size_t numTensorElements(nvinfer1::Dims dimensions);


  private:

    template <typename T>
    using UniquePtr = std::unique_ptr<T, InferDeleter>;

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    image_transport::ImageTransport image_transport_;

    image_transport::Publisher image_pub_;

    Logger g_logger_;

    std::string engine_file_path_ = "";

    std::string label_file_path_ = "";

    std::string input_name_ = "";

    std::string output_name_ = "";

    std::vector<float> mean_{0.485, 0.456, 0.406};

    std::vector<float> std_{0.229, 0.224, 0.225};

    std::vector<std::string> labels_;

    UniquePtr<nvinfer1::IRuntime> runtime_;

    UniquePtr<nvinfer1::ICudaEngine> engine_;

    UniquePtr<nvinfer1::IExecutionContext> context_;

    int input_binding_index_;

    int output_binding_index_;
  
    // nchw
    nvinfer1::Dims input_dims_;

    nvinfer1::Dims output_dims_;

    size_t num_input_;

    size_t num_output_;


  };

}  // namespace traffic_light
