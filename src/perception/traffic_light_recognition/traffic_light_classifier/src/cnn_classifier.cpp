/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "traffic_light_classifier/cnn_classifier.hpp"

namespace traffic_light {
CNNClassifier::CNNClassifier() : nh_(""), pnh_("~"), image_transport_(pnh_) {
  image_pub_ = image_transport_.advertise("output/debug/image", 1);

  std::string precision;
  std::string label_file_path;
  std::string model_file_path;
  pnh_.param<std::string>("precision", precision, "fp16");
  pnh_.param<std::string>("label_file_path", label_file_path, "labels.txt");
  pnh_.param<std::string>("model_file_path", model_file_path, "model.onnx");

  readLabelfile(label_file_path, labels_);

  trt_ = std::make_shared<Tn::TrtCommon>(model_file_path);
  trt_->precision = precision;
  trt_->cache_dir = ros::package::getPath("traffic_light_classifier") + "/data";
  trt_->setup();
}

bool CNNClassifier::getLampState(const cv::Mat& input_image,
                                 std::vector<autoware_traffic_light_msgs::LampState>& states) {
  if ( !trt_->is_initialized ) {
    ROS_WARN("failed to init tensorrt");
    return false;
  }

  float *input_data_host = (float*) malloc(trt_->num_input * sizeof(float));

  cv::Mat image = input_image.clone();
  preProcess(image, input_data_host, true);

  float *input_data_device;
  cudaMalloc((void**)&input_data_device,
             trt_->num_input * sizeof(float));
  cudaMemcpy(input_data_device,
             input_data_host,
             trt_->num_input * sizeof(float),
             cudaMemcpyHostToDevice);

  float *output_data_device;
  cudaMalloc((void**)&output_data_device,
             trt_->num_output * sizeof(float));

  // do inference
  void *bindings[2];
  bindings[trt_->input_binding_index] = (void*)input_data_device;
  bindings[trt_->output_binding_index] = (void*)output_data_device;

  std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
  trt_->context->executeV2(bindings);
  std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
  double elapsed_time = static_cast<double>
    (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) / 1000;
  // ROS_INFO("inference elapsed time: %f [ms]", elapsed_time);

  float *output_data_host = (float*) malloc(trt_->num_output * sizeof(float));
  cudaMemcpy(output_data_host,
             output_data_device,
             trt_->num_output * sizeof(float),
             cudaMemcpyDeviceToHost);

  postProcess(output_data_host, states);

  /* debug */
  if (0 < image_pub_.getNumSubscribers()) {
    cv::Mat debug_image = input_image.clone();
    outputDebugImage(debug_image, states);
  }

  cudaFree(input_data_device);
  cudaFree(output_data_device);

  return true;
}

void CNNClassifier::outputDebugImage(cv::Mat& debug_image,
                                     const std::vector<autoware_traffic_light_msgs::LampState>& states)
{
  float probability;
  std::string label;
  for (auto state : states) {
    // all lamp confidence are the same
    probability = state.confidence;
    label += state2label_[state.type];
  }

  int expand_w = 200;
  int expand_h = static_cast<int>((expand_w * debug_image.rows) / debug_image.cols);

  cv::resize(debug_image, debug_image, cv::Size(expand_w, expand_h));
  cv::Mat text_img(cv::Size(expand_w, 50), CV_8UC3, cv::Scalar(0,0,0));
  std::string text = label + " " + std::to_string(probability);
  cv::putText(text_img,
              text,
              cv::Point(5, 25),
              cv::FONT_HERSHEY_COMPLEX,
              0.5,
              cv::Scalar(0,255,0), 1);
  cv::vconcat(debug_image, text_img, debug_image);

  sensor_msgs::ImagePtr debug_image_msg = cv_bridge::CvImage
    (std_msgs::Header(), "bgr8", debug_image).toImageMsg();
  image_pub_.publish(debug_image_msg);
}

void CNNClassifier::preProcess(cv::Mat & image,
                               float *input_tensor,
                               bool normalize)
{
  /* normalize */
  /* ((channel[0] / 255) - mean[0]) / std[0] */

  const size_t channels = trt_->input_dims.d[1];
  const size_t height = trt_->input_dims.d[2];
  const size_t width = trt_->input_dims.d[3];

  cv::cvtColor(image, image, cv::COLOR_BGR2RGB, 3);
  cv::resize(image, image, cv::Size(width, height));

  const size_t stridesCv[3] = { width * channels, channels, 1 };
  const size_t strides[3] = { height * width, width, 1 };

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      for (int k = 0; k < channels; k++) {
        const size_t offsetCv = i * stridesCv[0] + j * stridesCv[1] + k * stridesCv[2];
        const size_t offset = k * strides[0] + i * strides[1] + j * strides[2];
        if (normalize) {
          input_tensor[offset] = (((float) image.data[offsetCv] / 255) - mean_[k]) / std_[k];
        } else {
          input_tensor[offset] = (float) image.data[offsetCv];
        }
      }
    }
  }

}

bool CNNClassifier::postProcess(float* output_tensor,
                                std::vector<autoware_traffic_light_msgs::LampState>& states)
{
  std::vector<float> probs;
  calcSoftmax(output_tensor, probs, trt_->num_output);
  std::vector<size_t> sorted_indices = argsort(output_tensor, trt_->num_output);

  // ROS_INFO("label: %s, score: %.2f\%",
  //          labels_[sorted_indices[0]].c_str(),
  //          probs[sorted_indices[0]] * 100);

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
                                std::vector<float>& probs,
                                int num_output)
{
  float exp_sum = 0.0;
  for (int i=0; i<num_output; ++i) {
    exp_sum += exp(data[i]);
  }

  for (int i=0; i<num_output; ++i) {
    probs.push_back(exp(data[i]) / exp_sum);
  }
}

std::vector<size_t> CNNClassifier::argsort(float *tensor,
                                           int num_output)
{
  std::vector<size_t> indices(num_output);
  for (int i = 0; i < num_output; i++)
    indices[i] = i;
  std::sort(indices.begin(), indices.begin() + num_output, [tensor](size_t idx1, size_t idx2) {
      return tensor[idx1] > tensor[idx2];
    });

  return indices;
}

}  // namespace traffic_light
