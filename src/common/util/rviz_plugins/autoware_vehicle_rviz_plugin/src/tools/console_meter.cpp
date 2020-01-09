#include "console_meter.hpp"
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <QPainter>

namespace rviz_plugins
{

std::unique_ptr<Ogre::ColourValue> ConsoleMeterDisplay::gradation(const QColor &color_min, const QColor &color_max,
                                                                  const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g = color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio);
  color_ptr->r = color_max.redF() * ratio + color_min.redF() * (1.0 - ratio);
  color_ptr->b = color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio);

  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> ConsoleMeterDisplay::setColorDependsOnVelocity(const double vel_max, const double cmd_vel)
{
  const double cmd_vel_abs = std::fabs(cmd_vel);
  const double vel_min = 0.0;

  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue());
  if (vel_min < cmd_vel_abs && cmd_vel_abs <= (vel_max / 2.0))
  {
    double ratio = (cmd_vel_abs - vel_min) / (vel_max / 2.0 - vel_min);
    color_ptr = gradation(Qt::red, Qt::yellow, ratio);
  }
  else if ((vel_max / 2.0) < cmd_vel_abs && cmd_vel_abs <= vel_max)
  {
    double ratio = (cmd_vel_abs - vel_max / 2.0) / (vel_max - vel_max / 2.0);
    color_ptr = gradation(Qt::yellow, Qt::green, ratio);
  }
  else if (vel_max < cmd_vel_abs)
  {
    *color_ptr = Ogre::ColourValue::Green;
  }
  else
  {
    *color_ptr = Ogre::ColourValue::Red;
  }

  return color_ptr;
}

ConsoleMeterDisplay::ConsoleMeterDisplay()
    : handle_image_("/home/tier4/saito/Autoware-T4B/src/common/util/rviz_plugins/autoware_vehicle_rviz_plugin/images/handle.png")
{
  property_text_color_ = new rviz::ColorProperty(
      "Text Color", QColor(25, 255, 240),
      "text color",
      this, SLOT(updateVisualization()), this);
  property_left_ = new rviz::IntProperty("left", 128,
                                         "left of the plotter window",
                                         this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz::IntProperty("top", 128,
                                        "top of the plotter window",
                                        this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_width_ = new rviz::IntProperty("width", 512,
                                          "width of the plotter window",
                                          this, SLOT(updateVisualization()), this);
  property_width_->setMin(1);
  property_height_ = new rviz::IntProperty("height", 128,
                                           "height of the plotter window",
                                           this, SLOT(updateVisualization()));
  property_height_->setMin(1);
}

ConsoleMeterDisplay::~ConsoleMeterDisplay()
{
  if (initialized())
  {
    // scene_manager_->destroyManualObject(path_manual_object_);
  }
}

void ConsoleMeterDisplay::onInitialize()
{
  MFDClass::onInitialize();
  static int count = 0;
  rviz::UniformStringStream ss;
  ss << "ConsoleMeterDisplayObject" << count++;
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(),
                              property_height_->getInt());
}

void ConsoleMeterDisplay::reset()
{
  MFDClass::reset();
}

void ConsoleMeterDisplay::processMessage(const autoware_control_msgs::VehicleStatusStampedConstPtr &msg_ptr)
{
  if (!isEnabled())
  {
    return;
  }
  if (!overlay_->isVisible())
  {
    return;
  }

  overlay_->updateTextureSize(property_width_->getInt(),
                              property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  QColor background_color;
  background_color.setAlpha(0);
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage Hud = buffer.getQImage(*overlay_);
  // initilize by the background color
  for (int i = 0; i < overlay_->getTextureWidth(); i++)
  {
    for (int j = 0; j < overlay_->getTextureHeight(); j++)
    {
      Hud.setPixel(i, j, background_color.rgba());
    }
  }

  QPainter painter(&Hud);
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor text_color(property_text_color_->getColor());
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, int(2), Qt::SolidLine));

  int w = overlay_->getTextureWidth();
  int h = overlay_->getTextureHeight();
  painter.drawLine(0, 0, 0, h);
  painter.drawLine(0, h, w, h);
  painter.drawLine(w, h, w, 0);
  painter.drawLine(w, 0, 0, 0);

  QFont font = painter.font();
  font.setPointSize(std::max(int(double(w) / 40.0), 1));
  font.setBold(true);
  painter.setFont(font);
  std::ostringstream steering_angle_ss;
  steering_angle_ss << std::fixed << std::setprecision(2) << msg_ptr->status.steering_angle * 180.0 / M_PI << "deg";
  painter.drawText(0, 0, w, h,
                   Qt::AlignCenter | Qt::AlignVCenter,
                   steering_angle_ss.str().c_str());

  QMatrix rotation_matrix;
  rotation_matrix.rotate(msg_ptr->status.steering_angle * -180.0);
  int handle_image_width = handle_image_.width(), handle_image_height = handle_image_.height();
  QPixmap rotate_handle_image;
  rotate_handle_image = handle_image_.transformed(rotation_matrix);
  rotate_handle_image = rotate_handle_image.copy((rotate_handle_image.width() - handle_image_width) / 2, (rotate_handle_image.height() - handle_image_height) / 2, handle_image_width, handle_image_height);
  painter.drawPixmap(10, 10, 100, 100, rotate_handle_image);
  // std::ostringstream velocity_ss;
  // velocity_ss << std::fixed << std::setprecision(2) << msg_ptr->status.velocity * 3.6 <<"km/h";
  // painter.drawText(0, 0, w, h,
  //                  Qt::AlignCenter | Qt::AlignVCenter,
  //                  velocity_ss.str().c_str());

  painter.end();
  last_msg_ptr_ = msg_ptr;
}

void ConsoleMeterDisplay::updateVisualization()
{
  overlay_->updateTextureSize(property_width_->getInt(),
                              property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  if (last_msg_ptr_ != nullptr)
    processMessage(last_msg_ptr_);
}

} // namespace rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ConsoleMeterDisplay, rviz::Display)
