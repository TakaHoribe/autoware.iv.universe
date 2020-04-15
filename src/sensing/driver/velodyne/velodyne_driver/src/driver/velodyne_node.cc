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
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Velodyne 3D LIDARs.
 */

#include <ros/ros.h>
#include "velodyne_driver/driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  velodyne_driver::VelodyneDriver dvr(node, private_nh);

  // loop until shut down or end of file
  while(ros::ok() && dvr.poll())
    {
      ros::spinOnce();
    }

  return 0;
}
