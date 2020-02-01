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

#include "autoware_vehicle_msgs/Steering.h"

#include "jsk_overlay_utils.hpp"
#endif

namespace rviz_plugins
{

class SteeringAngleDisplay : public rviz::MessageFilterDisplay<autoware_vehicle_msgs::Steering>
{
  Q_OBJECT

public:
  SteeringAngleDisplay();
  virtual ~SteeringAngleDisplay();

  void onInitialize() override;
  void onDisable() override;
  void onEnable() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const autoware_vehicle_msgs::SteeringConstPtr &msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(const QColor &color_min, const QColor &color_max,
                                               const double ratio);
  jsk_rviz_plugins::OverlayObject::Ptr overlay_;
  rviz::ColorProperty *property_text_color_;
  rviz::IntProperty *property_left_;
  rviz::IntProperty *property_top_;
  rviz::IntProperty *property_length_;
  rviz::FloatProperty *property_handle_angle_scale_;
  rviz::IntProperty *property_value_height_offset_;
  QPixmap handle_image_;
  // QImage hud_;

private:
  autoware_vehicle_msgs::SteeringConstPtr last_msg_ptr_;
};

} // namespace rviz_plugins
