/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <string>

#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>

std::string toString(const geometry_msgs::Pose& pose);
void setColor(std_msgs::ColorRGBA* cl, double r, double g, double b, double a);
void insertMarkerArray(visualization_msgs::MarkerArray* a1, const visualization_msgs::MarkerArray& a2);
bool getClosestLanelet(const geometry_msgs::Pose& search_pose, const lanelet::LaneletMapPtr& lanelet_map,
                       lanelet::Lanelet* closest_lanelet, double distance_thresh = 10.0);
