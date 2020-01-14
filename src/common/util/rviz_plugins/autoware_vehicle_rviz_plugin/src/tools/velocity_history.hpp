#pragma once

#include <ros/ros.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>

#include <deque>
#include <memory>
#include <tuple>

#include "autoware_control_msgs/VehicleStatusStamped.h"

namespace rviz_plugins
{

class VelocityHistoryDisplay : public rviz::MessageFilterDisplay<autoware_control_msgs::VehicleStatusStamped>
{
  Q_OBJECT

public:
  VelocityHistoryDisplay();
  virtual ~VelocityHistoryDisplay();

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const autoware_control_msgs::VehicleStatusStampedConstPtr &msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(const QColor &color_min, const QColor &color_max,
                                               const double ratio);
  Ogre::ManualObject *velocity_manual_object_;
  rviz::FloatProperty *property_velocity_timeout_;
  rviz::FloatProperty *property_path_alpha_;
  rviz::FloatProperty *property_velocity_alpha_;
  rviz::FloatProperty *property_velocity_scale_;
  rviz::BoolProperty *property_velocity_color_view_;
  rviz::ColorProperty *property_velocity_color_;
  rviz::FloatProperty *property_vel_max_;

private:
  std::deque<std::tuple<autoware_control_msgs::VehicleStatusStampedConstPtr, Ogre::Vector3>> history_;
  bool validateFloats(const autoware_control_msgs::VehicleStatusStampedConstPtr &msg_ptr);
};

} // namespace rviz_plugins
