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

#include <lane_change_planner/lane_changer.h>
#include <lane_change_planner/utilities.h>
namespace lane_change_planner
{
LaneChanger::LaneChanger() : pnh_("~")
{
}

void LaneChanger::init()
{
  timer_ = nh_.createTimer(ros::Duration(0.1), &LaneChanger::run, this);

  // data_manager
  points_subscriber_ =
      nh_.subscribe("points", 1, &SingletonDataManager::pointcloudCallback, &SingletonDataManager::getInstance());
  velocity_subscriber_ =
      nh_.subscribe("velocity", 1, &SingletonDataManager::velocityCallback, &SingletonDataManager::getInstance());
  perception_subscriber_ =
      nh_.subscribe("perception", 1, &SingletonDataManager::perceptionCallback, &SingletonDataManager::getInstance());

  // route_handler
  vector_map_subscriber_ = nh_.subscribe("vector_map", 1, &RouteHandler::mapCallback, &RouteHandler::getInstance());
  route_subscriber_ = nh_.subscribe("route", 1, &RouteHandler::routeCallback, &RouteHandler::getInstance());
  route_init_subscriber_ = nh_.subscribe("route", 1, &StateMachine::init, &state_machine_);

  // path_publisher
  path_publisher_ = nh_.advertise<autoware_planning_msgs::PathWithLaneId>("lane_change_path", 1);
}

void LaneChanger::run(const ros::TimerEvent& event)
{
  if (!RouteHandler::getInstance().isHandlerReady())
  {
    return;
  }

  state_machine_.updateState();
  auto path = state_machine_.getPath();
  auto goal = RouteHandler::getInstance().getGoalPose();
  auto goal_lane_id = RouteHandler::getInstance().getGoalLaneId();
  auto refined_path = util::refinePath(7.5, M_PI * 0.5, path, goal, goal_lane_id);
  // auto path = path_extender.ExtendPath(path);
  if (!path.points.empty())
  {
    path_publisher_.publish(refined_path);
  }
}

}  // namespace lane_change_planner
