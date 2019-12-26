#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include "new_trafficlight_recognizer/tlr_detector.h"

namespace new_trafficlight_recognizer
{
TrafficLightDetectorNode::TrafficLightDetectorNode() : nh_(), pnh_("~")
{
  std::string package_path = ros::package::getPath("new_trafficlight_recognizer");
  std::string engine_path = package_path + "/data/yolov3_tlr.engine";
  std::ifstream fs(engine_path);
  if (fs.is_open())
  {
    net_ptr_.reset(new Tn::trtNet(engine_path));
  }
  else
  {
    ROS_INFO("Could not find %s, try making TensorRT engine from onnx", engine_path.c_str());
    boost::filesystem::create_directories(package_path + "/data");
    std::string onnx_file;
    pnh_.param<std::string>("onnx_file", onnx_file, "");
    std::vector<std::vector<float>> calib_data;
    Tn::RUN_MODE run_mode = Tn::RUN_MODE::FLOAT32;
    net_ptr_.reset(new Tn::trtNet(onnx_file, calib_data, run_mode));
    net_ptr_->saveEngine(engine_path);
  }
  utils_ptr_ = std::make_shared<Utils>();
  initROS();
}

TrafficLightDetectorNode::~TrafficLightDetectorNode()
{
}

void TrafficLightDetectorNode::initROS()
{
  pnh_.param<bool>("approximate_sync", is_approximate_sync_, false);
  pnh_.param<double>("score_thresh", score_thresh_, 0.7);
  output_rois_pub_ = pnh_.advertise<autoware_traffic_light_msgs::StampedRoi>("output_stamped_roi", 1);
  output_debug_pub_ = pnh_.advertise<sensor_msgs::Image>("output_debug_image", 1);
  image_sub_.subscribe(pnh_, "input_image", 1);
  stamped_roi_sub_.subscribe(pnh_, "input_stamped_roi", 1);
  if (is_approximate_sync_)
  {
    approximate_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(1000);
    approximate_sync_->connectInput(image_sub_, stamped_roi_sub_);
    approximate_sync_->registerCallback(boost::bind(&TrafficLightDetectorNode::callback, this, _1, _2));
  }
  else
  {
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(1000);
    sync_->connectInput(image_sub_, stamped_roi_sub_);
    sync_->registerCallback(boost::bind(&TrafficLightDetectorNode::callback, this, _1, _2));
  }
}

void TrafficLightDetectorNode::callback(const sensor_msgs::Image::ConstPtr& in_image_msg,
                                        const autoware_traffic_light_msgs::StampedRoi::ConstPtr& in_roi_msg)
{
  cv::Mat original_image;
  cv::Mat debug_image;
  autoware_traffic_light_msgs::StampedRoi out_rois;
  std::vector<cv::Rect> out_rects;

  utils_ptr_->rosmsg2cvmat(in_image_msg, original_image);
  utils_ptr_->rosmsg2cvmat(in_image_msg, debug_image);
  int outputCount = net_ptr_->getOutputSize() / sizeof(float);
  std::unique_ptr<float[]> output_data(new float[outputCount]);
  size_t i = 0;
  int class_num = 1;

  for (const auto& roi : in_roi_msg->roi_array)
  {
    int signal = in_roi_msg->signals[i];
    cv::Point lt = cv::Point(roi.x_offset, roi.y_offset);
    cv::Point rb = cv::Point(roi.x_offset + roi.width, roi.y_offset + roi.height);
    utils_ptr_->fit_in_frame(lt, rb, cv::Size(original_image.size()));
    cv::Mat cropped(original_image, cv::Rect(lt, rb));
    cv::rectangle(debug_image, lt, rb, cv::Scalar(0, 255, 0), 2);
    std::vector<float> input_data = prepareImage(cropped);
    net_ptr_->doInference(input_data.data(), output_data.get());

    // Get Output
    auto output = output_data.get();
    auto bboxes = postProcessImg(output, class_num, cropped);
    if (bboxes.size() == 0)
      continue;
    for (const auto& item : bboxes)
    {
      if (item.score > score_thresh_)
      {
        cv::Point lt_roi = cv::Point(lt.x + item.left, lt.y + item.top);
        cv::Point rb_roi = cv::Point(lt.x + item.right, lt.y + item.bot);
        utils_ptr_->fit_in_frame(lt_roi, rb_roi, cv::Size(original_image.size()));
        cv::rectangle(debug_image, lt_roi, rb_roi, cv::Scalar(0, 0, 255), 2);
        out_rois.signals.push_back(signal);
        out_rects.push_back(cv::Rect(lt_roi, rb_roi));
        break;
      }
    }
    ++i;
  }
  out_rois.header = in_roi_msg->header;
  utils_ptr_->cvrects2roismsg(out_rects, out_rois.roi_array);
  output_rois_pub_.publish(out_rois);
  output_debug_pub_.publish(
      cv_bridge::CvImage(in_image_msg->header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg());
}

std::vector<float> TrafficLightDetectorNode::prepareImage(cv::Mat& in_img)
{
  // using namespace cv;

  int c = 3;
  int h = 256;
  int w = 256;

  float scale = std::min(float(w) / in_img.cols, float(h) / in_img.rows);
  auto scaleSize = cv::Size(in_img.cols * scale, in_img.rows * scale);

  cv::Mat rgb;
  cv::cvtColor(in_img, rgb, CV_BGR2RGB);
  cv::Mat resized;
  cv::resize(rgb, resized, scaleSize, 0, 0, cv::INTER_CUBIC);

  cv::Mat cropped(h, w, CV_8UC3, 127);
  cv::Rect rect((w - scaleSize.width) / 2, (h - scaleSize.height) / 2, scaleSize.width, scaleSize.height);
  resized.copyTo(cropped(rect));

  cv::Mat img_float;
  cropped.convertTo(img_float, CV_32FC3, 1 / 255.0);

  // HWC TO CHW
  std::vector<cv::Mat> input_channels(c);
  cv::split(img_float, input_channels);

  std::vector<float> result(h * w * c);
  auto data = result.data();
  int channelLength = h * w;
  for (int i = 0; i < c; ++i)
  {
    memcpy(data, input_channels[i].data, channelLength * sizeof(float));
    data += channelLength;
  }

  return result;
}

std::vector<Tn::Bbox> TrafficLightDetectorNode::postProcessImg(float* output, const int classes, cv::Mat& img)
{
  int detect_h = 256;
  int detect_w = 256;
  float obj_threshold = 0.10;
  std::vector<Detection> detections;
  int total_size = 0;
  for (size_t i = 0; i < output_shape.size(); i++)
  {
    auto shape = output_shape[i];
    int size = 1;
    for (size_t j = 0; j < shape.size(); j++)
    {
      size *= shape[j];
    }
    total_size += size;
  }
  int offset = 0;
  float* transposed_output = new float[total_size];
  float* transposed_output_t = transposed_output;
  for (size_t i = 0; i < output_shape.size(); i++)
  {
    auto shape = output_shape[i];  // nchw
    int chw = shape[1] * shape[2] * shape[3];
    int hw = shape[2] * shape[3];
    for (int n = 0; n < shape[0]; n++)
    {
      int offset_n = offset + n * chw;
      for (int h = 0; h < shape[2]; h++)
      {
        for (int w = 0; w < shape[3]; w++)
        {
          int h_w = h * shape[3] + w;
          for (int c = 0; c < shape[1]; c++)
          {
            int offset_c = offset_n + hw * c + h_w;
            *transposed_output_t++ = output[offset_c];
          }
        }
      }
    }
    offset += shape[0] * chw;
  }
  std::vector<std::vector<int>> shapes;
  for (size_t i = 0; i < output_shape.size(); i++)
  {
    auto shape = output_shape[i];
    std::vector<int> tmp = { shape[2], shape[3], 3, 6 };
    shapes.push_back(tmp);
  }

  offset = 0;
  for (size_t i = 0; i < output_shape.size(); i++)
  {
    auto masks = g_masks[i];
    std::vector<std::vector<int>> anchors;
    for (auto mask : masks)
      anchors.push_back(g_anchors[mask]);
    auto shape = shapes[i];
    for (int h = 0; h < shape[0]; h++)
    {
      int offset_h = offset + h * shape[1] * shape[2] * shape[3];
      for (int w = 0; w < shape[1]; w++)
      {
        int offset_w = offset_h + w * shape[2] * shape[3];
        for (int c = 0; c < shape[2]; c++)
        {
          int offset_c = offset_w + c * shape[3];
          float* ptr = transposed_output + offset_c;
          ptr[4] = sigmoid(ptr[4]);
          ptr[5] = sigmoid(ptr[5]);
          float score = ptr[4] * ptr[5];
          if (score < obj_threshold)
            continue;
          ptr[0] = sigmoid(ptr[0]);
          ptr[1] = sigmoid(ptr[1]);
          ptr[2] = std::exp(ptr[2]) * anchors[c][0];
          ptr[3] = std::exp(ptr[3]) * anchors[c][1];

          ptr[0] += w;
          ptr[1] += h;
          ptr[0] /= shape[0];
          ptr[1] /= shape[1];
          ptr[2] /= detect_w;
          ptr[3] /= detect_h;
          ptr[0] -= ptr[2] / 2;
          ptr[1] -= ptr[3] / 2;

          Detection det;
          det.x = ptr[0];
          det.y = ptr[1];
          det.w = ptr[2];
          det.h = ptr[3];
          det.prob = score;
          detections.push_back(det);
        }
      }
    }
    offset += shape[0] * shape[1] * shape[2] * shape[3];
  }
  delete[] transposed_output;

  // scale bbox to img
  int width = img.cols;
  int height = img.rows;
  float scale = std::min(float(detect_w) / width, float(detect_h) / height);
  float scaleSize[] = { width * scale, height * scale };

  // correct box
  for (auto& det : detections)
  {
    det.x = (det.x * detect_w - (detect_w - scaleSize[0]) / 2.f) / scale;
    det.y = (det.y * detect_h - (detect_h - scaleSize[1]) / 2.f) / scale;
    det.w *= detect_w;
    det.h *= detect_h;
    det.w /= scale;
    det.h /= scale;
  }

  // nms
  float nms_thresh = 0.45;
  if (nms_thresh > 0)
    doNms(detections, nms_thresh);
  std::vector<Tn::Bbox> boxes;
  for (const auto& det : detections)
  {
    Tn::Bbox bbox = {
      0,                                     // classId
      std::max(int(det.x), 0),               // left
      std::min(int(det.x + det.w), width),   // right
      std::max(int(det.y), 0),               // top
      std::min(int(det.y + det.h), height),  // bot
      det.prob,                              // score
    };
    boxes.push_back(bbox);
  }
  return boxes;
}

void TrafficLightDetectorNode::doNms(std::vector<Detection>& detections, float nms_thresh)
{
  auto iouCompute = [](float* lbox, float* rbox) {
    float interBox[] = {
      std::max(lbox[0], rbox[0]),                      // left
      std::min(lbox[0] + lbox[2], rbox[0] + rbox[2]),  // right
      std::max(lbox[1], rbox[1]),                      // top
      std::min(lbox[1] + lbox[3], rbox[1] + rbox[3]),  // bottom
    };

    if (interBox[2] >= interBox[3] || interBox[0] >= interBox[1])
      return 0.0f;

    float interBoxS = (interBox[1] - interBox[0] + 1) * (interBox[3] - interBox[2] + 1);
    return interBoxS / (lbox[2] * lbox[3] + rbox[2] * rbox[3] - interBoxS);
  };

  sort(detections.begin(), detections.end(),
       [=](const Detection& left, const Detection& right) { return left.prob > right.prob; });

  std::vector<Detection> result;
  for (unsigned int m = 0; m < detections.size(); ++m)
  {
    result.push_back(detections[m]);
    for (unsigned int n = m + 1; n < detections.size(); ++n)
    {
      if (iouCompute((float*)(&detections[m]), (float*)(&detections[n])) > nms_thresh)
      {
        detections.erase(detections.begin() + n);
        --n;
      }
    }
  }
  detections = move(result);
}

void TrafficLightDetectorNode::run()
{
  ros::spin();
}
}  // namespace
