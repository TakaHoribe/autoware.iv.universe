#include "new_trafficlight_recognizer/visualizer_node.h"

namespace new_trafficlight_recognizer
{

  void TrafficLightVisualizerNode::callback
  (const sensor_msgs::Image::ConstPtr& image_msg,
   const autoware_traffic_light_msgs::StampedRoi::ConstPtr& projected_roi_msg,
   const autoware_traffic_light_msgs::StampedRoi::ConstPtr& tracker_roi_msg,
   const autoware_msgs::Signals::ConstPtr& signals_msg,
   const autoware_traffic_light_msgs::TrafficLightStateArray::ConstPtr& light_state_msg)
  {

    cv::Mat image;
    utils_->rosmsg2cvmat(image_msg, image);

    // generate tracker roi map
    std::vector<cv::Rect> tracker_rects;
    utils_->roismsg2cvrects(tracker_roi_msg->roi_array, tracker_rects);
    std::map<int, cv::Rect> tracker_roi_map;
    for (int i=0; i<tracker_roi_msg->signals.size(); ++i)
      {
        int signal = tracker_roi_msg->signals.at(i);
        sensor_msgs::RegionOfInterest roi = tracker_roi_msg->roi_array.at(i);
        tracker_roi_map.emplace(signal,
                                cv::Rect(roi.x_offset,
                                         roi.y_offset,
                                         roi.width,
                                         roi.height));
      }

    // generate projected roi map
    std::map<int, cv::Rect> projected_roi_map;
    std::vector<cv::Rect> projected_rects;
    utils_->roismsg2cvrects(projected_roi_msg->roi_array, projected_rects);
    for (int i=0; i<projected_roi_msg->signals.size(); ++i)
      {
        int signal = projected_roi_msg->signals.at(i);
        if (utils_->in_vector(tracker_roi_msg->signals, signal) )
          {
            sensor_msgs::RegionOfInterest roi = projected_roi_msg->roi_array.at(i);
            projected_roi_map.emplace(signal,
                                      cv::Rect(roi.x_offset,
                                               roi.y_offset,
                                               roi.width,
                                               roi.height));
          }
      }

    // generate pos3d map
    std::map<int, geometry_msgs::Point> pos3d_map;
    for (int i=0; i<signals_msg->Signals.size(); ++i)
      {
        autoware_msgs::ExtractedPosition signal_msg = signals_msg->Signals.at(i);
        int signal = signal_msg.signalId;

        if (utils_->in_vector(tracker_roi_msg->signals, signal) )
          {
            geometry_msgs::Point p;
            p.x = signal_msg.x;
            p.y = signal_msg.y;
            p.z = signal_msg.z;
            pos3d_map.emplace(signal, p);
          }
      }

    // generate lamp state map
    std::map<int, std::vector<autoware_traffic_light_msgs::LampState> > lamp_states_map;
    for (int i=0; i<signals_msg->Signals.size(); ++i)
      {
        autoware_msgs::ExtractedPosition signal_msg = signals_msg->Signals.at(i);
        int signal = signal_msg.signalId;

        for (auto light_state : light_state_msg->states)
          {
            if (signal == light_state.signal)
              {
                lamp_states_map.emplace(signal, light_state.states);
              }
          }
      }


    if (tracker_roi_map.size() != projected_roi_map.size() ||
        tracker_roi_map.size() != pos3d_map.size())
        return;


    visualization_msgs::MarkerArray marker_array;
    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (auto it = tracker_roi_map.begin(); it != tracker_roi_map.end(); ++it)
      {
        // visualize rects on image
        int signal = it->first;
        cv::rectangle(image, tracker_roi_map.at(signal), CV_RGB(0,255,0), 2);
        cv::rectangle(image, projected_roi_map.at(signal), CV_RGB(0,0,255), 2);

        // visualize classification result on image
        for (auto lamp_state : lamp_states_map.at(signal))
          {
            // TODO visualize lamp state on image
          }


        // visualize on rviz
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.pose.position.x = pos3d_map.at(signal).x;
        bbox.pose.position.y = pos3d_map.at(signal).y;
        bbox.pose.position.z = pos3d_map.at(signal).z;
        bbox.dimensions.x = 2;
        bbox.dimensions.y = 2;
        bbox.dimensions.z = 0.7;
        bbox.header = image_msg->header;
        bbox_array.boxes.push_back(bbox);

        // vis line on rviz
        visualization_msgs::Marker marker;
        marker.header = image_msg->header;
        marker.ns = "visualization_lines_" + std::to_string(signal);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 2;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.1;
        marker.color.g = 1.0;
        marker.color.a = 1.0;

        int split = 100;
        for (uint32_t i = 0; i < split; ++i) {
          geometry_msgs::Point p;
          p.x = pos3d_map.at(signal).x * i / split;
          p.y = pos3d_map.at(signal).y * i / split;
          p.z = pos3d_map.at(signal).z * i / split;

          marker.points.push_back(p);
        }
        marker_array.markers.push_back(marker);
      }

    bbox_array.header = image_msg->header;

    visualization_lines_pub_.publish(marker_array);
    visualization_bboxes_pub_.publish(bbox_array);
    visualization_image_pub_.publish(cv_bridge::CvImage(image_msg->header,
                                                        sensor_msgs::image_encodings::BGR8,
                                                        image).toImageMsg());

  }

  void TrafficLightVisualizerNode::run()
  {
    ros::NodeHandle pnh("~");

    pnh.getParam("approximate_sync", is_approximate_sync_);

    visualization_image_pub_ =
      pnh.advertise<sensor_msgs::Image>("output_vis_image", 1);
    visualization_lines_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("output_vis_lines", 1);
    visualization_bboxes_pub_ =
      pnh.advertise<jsk_recognition_msgs::BoundingBoxArray>("output_vis_bboxes", 1);

    image_sub_.subscribe(pnh, "input_image", 1);
    projected_roi_sub_.subscribe(pnh, "input_projected_roi", 1);
    tracker_roi_sub_.subscribe(pnh, "input_tracker_roi", 1);
    signals_sub_.subscribe(pnh, "input_signals", 1);
    light_state_sub_.subscribe(pnh, "input_light_state", 1);


    if (is_approximate_sync_)
      {
        approximate_sync_ =
          boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
        approximate_sync_->connectInput(image_sub_, projected_roi_sub_, tracker_roi_sub_, signals_sub_, light_state_sub_);
        approximate_sync_->registerCallback
          (boost::bind(&TrafficLightVisualizerNode::callback, this, _1, _2, _3, _4, _5));
      }
    else
      {
        sync_ =
          boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        sync_->connectInput(image_sub_, projected_roi_sub_, tracker_roi_sub_, signals_sub_, light_state_sub_);
        sync_->registerCallback
          (boost::bind(&TrafficLightVisualizerNode::callback, this, _1, _2, _3, _4, _5));
      }

    ros::spin();

  } // TrafficLightVisualizerNode 

} // new_trafficlight_recognizer


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trafficlight_visualizer");

  new_trafficlight_recognizer::TrafficLightVisualizerNode visualizer;
  visualizer.run();

  return 0;
}
