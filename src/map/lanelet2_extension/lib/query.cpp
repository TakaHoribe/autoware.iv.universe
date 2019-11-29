/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 */

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>

#include <set>
#include <string>
#include <vector>

namespace lanelet
{
namespace utils
{
// returns all lanelets in laneletLayer - don't know how to convert
// PrimitveLayer<Lanelets> -> std::vector<Lanelets>
lanelet::ConstLanelets query::laneletLayer(const lanelet::LaneletMapPtr ll_map)
{
  lanelet::ConstLanelets lanelets;
  if (!ll_map)
  {
    ROS_WARN("No map received!");
    return lanelets;
  }

  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++)
  {
    lanelets.push_back(*li);
  }

  return lanelets;
}

lanelet::ConstLanelets query::subtypeLanelets(const lanelet::ConstLanelets lls, const char subtype[])
{
  lanelet::ConstLanelets subtype_lanelets;

  for (auto li = lls.begin(); li != lls.end(); li++)
  {
    lanelet::ConstLanelet ll = *li;

    if (ll.hasAttribute(lanelet::AttributeName::Subtype))
    {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype)
      {
        subtype_lanelets.push_back(ll);
      }
    }
  }

  return subtype_lanelets;
}

lanelet::ConstLanelets query::crosswalkLanelets(const lanelet::ConstLanelets lls)
{
  return (query::subtypeLanelets(lls, lanelet::AttributeValueString::Crosswalk));
}

lanelet::ConstLanelets query::roadLanelets(const lanelet::ConstLanelets lls)
{
  return (query::subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}

std::vector<lanelet::TrafficLightConstPtr> query::trafficLights(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<lanelet::TrafficLightConstPtr> ll_tl_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++)
    {
      lanelet::TrafficLightConstPtr tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;
      for (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }
      if (unique_id)
      {
        tl_reg_elems.push_back(tl_ptr);
      }
    }
  }
  return tl_reg_elems;
}

std::vector<lanelet::AutowareTrafficLightConstPtr> query::autowareTrafficLights(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<lanelet::AutowareTrafficLightConstPtr> ll_tl_re =
        ll.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++)
    {
      lanelet::AutowareTrafficLightConstPtr tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;

      for (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }

      if (unique_id)
        tl_reg_elems.push_back(tl_ptr);
    }
  }
  return tl_reg_elems;
}

std::vector<lanelet::DetectionAreaConstPtr> query::detectionAreas(const lanelet::ConstLanelets& lanelets)
{
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<lanelet::DetectionAreaConstPtr> ll_da_re = ll.regulatoryElementsAs<lanelet::autoware::DetectionArea>();

    // insert unique tl into array
    for (const auto& da_ptr : ll_da_re)
    {
      lanelet::Id id = da_ptr->id();
      bool unique_id = true;

      for (auto ii = da_reg_elems.begin(); ii != da_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }

      if (unique_id)
        da_reg_elems.push_back(da_ptr);
    }
  }
  return da_reg_elems;
}

// return all stop lines and ref lines from a given set of lanelets
std::vector<lanelet::ConstLineString3d> query::stopLinesLanelets(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  for (auto lli = lanelets.begin(); lli != lanelets.end(); lli++)
  {
    std::vector<lanelet::ConstLineString3d> ll_stoplines;
    ll_stoplines = query::stopLinesLanelet(*lli);
    stoplines.insert(stoplines.end(), ll_stoplines.begin(), ll_stoplines.end());
  }

  return stoplines;
}

// return all stop and ref lines from a given lanel
std::vector<lanelet::ConstLineString3d> query::stopLinesLanelet(const lanelet::ConstLanelet ll)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  // find stop lines referened by right ofway reg. elems.
  std::vector<std::shared_ptr<const lanelet::RightOfWay> > right_of_way_reg_elems =
      ll.regulatoryElementsAs<const lanelet::RightOfWay>();

  if (right_of_way_reg_elems.size() > 0)
  {
    // lanelet has a right of way elem elemetn
    for (auto j = right_of_way_reg_elems.begin(); j < right_of_way_reg_elems.end(); j++)
    {
      if ((*j)->getManeuver(ll) == lanelet::ManeuverType::Yield)
      {
        // lanelet has a yield reg. elem.
        lanelet::Optional<lanelet::ConstLineString3d> row_stopline_opt = (*j)->stopLine();
        if (!!row_stopline_opt)
          stoplines.push_back(row_stopline_opt.get());
      }
    }
  }

  // find stop lines referenced by traffic lights
  std::vector<std::shared_ptr<const lanelet::TrafficLight> > traffic_light_reg_elems =
      ll.regulatoryElementsAs<const lanelet::TrafficLight>();

  if (traffic_light_reg_elems.size() > 0)
  {
    // lanelet has a traffic light elem elemetn
    for (auto j = traffic_light_reg_elems.begin(); j < traffic_light_reg_elems.end(); j++)
    {
      lanelet::Optional<lanelet::ConstLineString3d> traffic_light_stopline_opt = (*j)->stopLine();
      if (!!traffic_light_stopline_opt)
        stoplines.push_back(traffic_light_stopline_opt.get());
    }
  }
  // find stop lines referenced by traffic signs
  std::vector<std::shared_ptr<const lanelet::TrafficSign> > traffic_sign_reg_elems =
      ll.regulatoryElementsAs<const lanelet::TrafficSign>();

  if (traffic_sign_reg_elems.size() > 0)
  {
    // lanelet has a traffic sign reg elem - can have multiple ref lines (but
    // stop sign shod have 1
    for (auto j = traffic_sign_reg_elems.begin(); j < traffic_sign_reg_elems.end(); j++)
    {
      lanelet::ConstLineStrings3d traffic_sign_stoplines = (*j)->refLines();
      if (traffic_sign_stoplines.size() > 0)
        stoplines.push_back(traffic_sign_stoplines.front());
    }
  }
  return stoplines;
}

std::vector<lanelet::ConstLineString3d> query::stopSignStopLines(const lanelet::ConstLanelets lanelets,
                                                                 const std::string& stop_sign_id)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  std::set<lanelet::Id> checklist;

  for (const auto& ll : lanelets)
  {
    // find stop lines referenced by traffic signs
    std::vector<std::shared_ptr<const lanelet::TrafficSign> > traffic_sign_reg_elems =
        ll.regulatoryElementsAs<const lanelet::TrafficSign>();

    if (traffic_sign_reg_elems.size() > 0)
    {
      // lanelet has a traffic sign reg elem - can have multiple ref lines (but
      // stop sign shod have 1
      for (const auto& ts : traffic_sign_reg_elems)
      {
        // skip if traffic sign is not stop sign
        if (ts->type() != stop_sign_id)
        {
          continue;
        }

        lanelet::ConstLineStrings3d traffic_sign_stoplines = ts->refLines();

        // only add new items
        if (traffic_sign_stoplines.size() > 0)
        {
          auto id = traffic_sign_stoplines.front().id();
          if (checklist.find(id) == checklist.end())
          {
            checklist.insert(id);
            stoplines.push_back(traffic_sign_stoplines.front());
          }
        }
      }
    }
  }
  return stoplines;
}

ConstLanelets query::getLaneletsWithinRange(const lanelet::ConstLanelets& lanelets, const lanelet::BasicPoint2d& search_point, const double range)
{
  ConstLanelets near_lanelets;
  for (const auto& ll : lanelets)
  {
    lanelet::BasicPolygon2d poly = ll.polygon2d().basicPolygon();
    double distance = lanelet::geometry::distance(poly, search_point);
    if (distance <= range)
    {
      near_lanelets.push_back(ll);
    }
  }
  return near_lanelets;
}

ConstLanelets query::getLaneletsWithinRange(const lanelet::ConstLanelets& lanelets, const geometry_msgs::Point& search_point, const double range)
{
  getLaneletsWithinRange(lanelets, lanelet::BasicPoint2d(search_point.x, search_point.y), range);
}

ConstLanelets query::getLaneChangeableNeighbors(const routing::RoutingGraphPtr& graph, const ConstLanelet& lanelet)
{
  return graph->besides(lanelet);
}

ConstLanelets query::getLaneChangeableNeighbors(const routing::RoutingGraphPtr& graph, const ConstLanelets& road_lanelets, const geometry_msgs::Point& search_point)
{
  const auto lanelets = getLaneletsWithinRange(road_lanelets, search_point, std::numeric_limits<double>::epsilon());
  ConstLanelets road_slices;
  for (const auto& llt : lanelets)
  {
    const auto tmp_road_slice = getLaneChangeableNeighbors(graph, llt);
    road_slices.insert(road_slices.end(), tmp_road_slice.begin(), tmp_road_slice.end()); 
  }
  return road_slices;
}


ConstLanelets query::getAllNeighbors(const routing::RoutingGraphPtr& graph, const ConstLanelet& lanelet)
{
  ConstLanelets lanelets;
  lanelets.push_back(lanelet);
  auto right_lane = (!!graph->right(lanelet)) ? graph->right(lanelet) : graph->adjacentRight(lanelet);
  while (!!right_lane)
  {
    lanelets.push_back(right_lane.get());
    right_lane =
        (!!graph->right(right_lane.get())) ? graph->right(right_lane.get()) : graph->adjacentRight(right_lane.get());
  }
  auto left_lane = (!!graph->left(lanelet)) ? graph->left(lanelet) : graph->adjacentLeft(lanelet);
  while (!!left_lane)
  {
    lanelets.push_back(left_lane.get());
    left_lane =
        (!!graph->left(left_lane.get())) ? graph->left(left_lane.get()) : graph->adjacentLeft(left_lane.get());
  }
  return lanelets;
}

ConstLanelets query::getAllNeighbors(const routing::RoutingGraphPtr& graph,
                                                const ConstLanelets& road_lanelets,
                                                const geometry_msgs::Point& search_point)
{
  const auto lanelets = getLaneletsWithinRange(road_lanelets, search_point, std::numeric_limits<double>::epsilon());
  ConstLanelets road_slices;
  for (const auto& llt : lanelets)
  {
    const auto tmp_road_slice = getAllNeighbors(graph, llt);
    road_slices.insert(road_slices.end(), tmp_road_slice.begin(), tmp_road_slice.end());
  }
  return road_slices;
}

}  // namespace utils
}  // namespace lanelet
