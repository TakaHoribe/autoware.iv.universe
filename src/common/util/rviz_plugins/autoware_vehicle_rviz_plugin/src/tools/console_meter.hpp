#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/message_filter_display.h>
// #include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>

#include <deque>
#include <memory>

#include "autoware_control_msgs/VehicleStatusStamped.h"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{

class ConsoleMeterDisplay : public rviz::MessageFilterDisplay<autoware_control_msgs::VehicleStatusStamped>
{
  Q_OBJECT

public:
  ConsoleMeterDisplay();
  virtual ~ConsoleMeterDisplay();

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const autoware_control_msgs::VehicleStatusStampedConstPtr &msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(const QColor &color_min, const QColor &color_max,
                                               const double ratio);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz::ColorProperty *property_text_color_;
  rviz::IntProperty *property_left_;
  rviz::IntProperty *property_top_;
  rviz::IntProperty *property_width_;
  rviz::IntProperty *property_height_;
  QPixmap handle_image_;

private:
  autoware_control_msgs::VehicleStatusStampedConstPtr last_msg_ptr_;
};

} // namespace rviz_plugins
