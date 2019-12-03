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

#include "autoware_planning_msgs/Path.h"

namespace rviz_plugins
{

class AutowarePathDisplay : public rviz::MessageFilterDisplay<autoware_planning_msgs::Path>
{
  Q_OBJECT

public:
  AutowarePathDisplay();
  virtual ~AutowarePathDisplay();

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateVisualization();

protected:
  void processMessage(const autoware_planning_msgs::PathConstPtr &msg_ptr) override;
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double vel_max, const double cmd_vel);
  std::unique_ptr<Ogre::ColourValue> gradation(const QColor &color_min, const QColor &color_max,
                                               const double ratio);
  Ogre::ManualObject *path_manual_object_;
  Ogre::ManualObject *velocity_manual_object_;
  rviz::BoolProperty *property_path_view_;
  rviz::BoolProperty *property_velocity_view_;
  rviz::FloatProperty *property_path_width_;
  rviz::ColorProperty *property_path_color_;
  rviz::ColorProperty *property_velocity_color_;
  rviz::FloatProperty *property_path_alpha_;
  rviz::FloatProperty *property_velocity_alpha_;
  rviz::BoolProperty *property_path_color_view_;
  rviz::BoolProperty *property_velocity_color_view_;
  rviz::FloatProperty *property_vel_max_;

private:
  autoware_planning_msgs::PathConstPtr last_msg_ptr_;
  bool validateFloats(const autoware_planning_msgs::PathConstPtr &msg_ptr);
};

} // namespace rviz_plugins
