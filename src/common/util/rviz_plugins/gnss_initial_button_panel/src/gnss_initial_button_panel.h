/*
 * Copyright (c) 2018, TierIV Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QSettings>
#include <QPushButton>
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "ndt_scan_matcher/NDTAlign.h"

namespace autoware_rviz_debug
{
class GNSSInitialButtonPanel : public rviz::Panel
{
  Q_OBJECT
public:
  GNSSInitialButtonPanel(QWidget* parent = 0);
  void callbackGNSSPoseCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

public Q_SLOTS:
  void pushInitialzeButton();

protected:
  ros::NodeHandle nh_;
  ros::Subscriber gnss_pose_cov_sub_;

  ros::ServiceClient client_;

  QPushButton* initialize_button_;
  QLabel* status_label_;

  geometry_msgs::PoseWithCovarianceStamped gnss_pose_cov_msg_;

};

}  // end namespace autoware_rviz_debug

