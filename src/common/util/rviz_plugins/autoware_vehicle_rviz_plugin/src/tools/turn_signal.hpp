#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/message_filter_display.h>
// #include <rviz/properties/ros_topic_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <deque>
#include <memory>

#include "autoware_vehicle_msgs/TurnSignal.h"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins {

class TurnSignalDisplay : public rviz::MessageFilterDisplay<autoware_vehicle_msgs::TurnSignal> {
  Q_OBJECT

 public:
  TurnSignalDisplay();
  virtual ~TurnSignalDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

 private Q_SLOTS:
  void updateVisualization();

 protected:
  void processMessage(const autoware_vehicle_msgs::TurnSignalConstPtr& msg_ptr) override;
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz::IntProperty* property_left_;
  rviz::IntProperty* property_top_;
  rviz::IntProperty* property_width_;
  rviz::IntProperty* property_height_;
  // QImage hud_;

 private:
  autoware_vehicle_msgs::TurnSignalConstPtr last_msg_ptr_;
};

}  // namespace rviz_plugins
