#include <scene_module/traffic_light/traffic_light.hpp>
#include <behavior_velocity_planner/api.hpp>
#include <map>

namespace behavior_planning
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Line = bg::model::linestring<Point>;
using Polygon = bg::model::polygon<Point, false>;

TrafficLightModule::TrafficLightModule(TrafficLightModuleManager *manager_ptr,
                                       const std::shared_ptr<lanelet::TrafficLight const> traffic_light_ptr,
                                       const int lane_id)
    : manager_ptr_(manager_ptr),
      traffic_light_ptr_(traffic_light_ptr),
      lane_id_(lane_id),
      stop_margin_(0.0),
      task_id_(boost::uuids::random_generator()())

{
    if (manager_ptr_ != nullptr)
        manager_ptr_->registerTask(*traffic_light_ptr, task_id_);
}

TrafficLightModule::~TrafficLightModule()
{
    if (manager_ptr_ != nullptr)
        manager_ptr_->unregisterTask(task_id_);
}

bool TrafficLightModule::run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output)
{
    output = input;
    lanelet::ConstLineString3d lanelet_stop_line = *(traffic_light_ptr_->stopLine());
    lanelet::ConstLineStringsOrPolygons3d traffic_lights = traffic_light_ptr_->trafficLights();

    // search traffic light state
    bool found = false;
    autoware_traffic_light_msgs::TrafficLightState tl_state;
    for (const auto &traffic_light:traffic_lights){
        if (!traffic_light.isLineString()) // traffic ligth must be linestrings
            continue;
        const int id = static_cast<lanelet::ConstLineString3d>(traffic_light).id();
        std_msgs::Header header;
        if (!getTrafficLightState(id, header, tl_state))
            continue;
        if (!((ros::Time::now() - header.stamp).toSec() < 1.0))
            continue;
        if (tl_state.lamp_states.empty() || tl_state.lamp_states.front().type == autoware_traffic_light_msgs::LampState::UNKNOWN)
            continue;

        found = true;
        break;
    }
    if (!found)
    {
        ROS_WARN_THROTTLE(1.0,"cannot find traffic light lamp state");
        return false;
    }

    Line stop_line = {{lanelet_stop_line[0].x(), lanelet_stop_line[0].y()},
                      {lanelet_stop_line[1].x(), lanelet_stop_line[1].y()}};

    // debug code
    // autoware_traffic_light_msgs::LampState lamp_state;
    // lamp_state.type = autoware_traffic_light_msgs::LampState::RED;
    // lamp_state.confidence = 1.0;
    // tl_state.lamp_states.push_back(lamp_state);
    // manager_ptr_->debuger.pushTrafficLightState(traffic_light_ptr_, tl_state);
    std::cout << __LINE__ << ":" << tl_state.lamp_states.size() << std::endl;
    std::cout << __LINE__ << ":" << tl_state.lamp_states.front().type << std::endl;

    // if state is red, insert stop point into path
    if (tl_state.lamp_states.front().type == autoware_traffic_light_msgs::LampState::RED)
    {
        if (!insertTargetVelocityPoint(input,
                                       stop_line,
                                       stop_margin_,
                                       0.0,
                                       output))
        {
            return false;
        }
    }
    return true;
}

bool TrafficLightModule::endOfLife(const autoware_planning_msgs::PathWithLaneId &input)
{
    bool is_end_of_life = false;
    bool found = false;
    for (size_t i = 0; i < input.points.size(); ++i)
    {
        for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j)
        {

            if (lane_id_ == input.points.at(i).lane_ids.at(j))
                found = true;
        }
    }

    is_end_of_life = !found;
    return is_end_of_life;
}

bool TrafficLightModule::insertTargetVelocityPoint(const autoware_planning_msgs::PathWithLaneId &input,
                                                   const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &stop_line,
                                                   const double &margin,
                                                   const double &velocity,
                                                   autoware_planning_msgs::PathWithLaneId &output)
{
    output = input;
    for (size_t i = 0; i < output.points.size() - 1; ++i)
    {
        Line path_line = {{output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y},
                          {output.points.at(i + 1).point.pose.position.x, output.points.at(i + 1).point.pose.position.y}};
        std::vector<Point> collision_points;
        bg::intersection(stop_line, path_line, collision_points);

        if (collision_points.empty())
            continue;

        // check nearest collision point
        Point nearest_collision_point;
        double min_dist;
        for (size_t j = 0; j < collision_points.size(); ++j)
        {
            double dist = bg::distance(Point(output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y),
                                       collision_points.at(j));
            if (j == 0 || dist < min_dist)
            {
                min_dist = dist;
                nearest_collision_point = collision_points.at(j);
            }
        }

        // search target point index
        size_t insert_target_point_idx;
        double base_link2front;
        double length_sum = 0;
        if (!getBaselink2FrontLength(base_link2front))
        {
            ROS_ERROR("cannot get vehicle front to base_link");
            return false;
        }
        const double target_length = margin + base_link2front;
        Eigen::Vector2d point1, point2;
        point1 << nearest_collision_point.x(), nearest_collision_point.y();
        point2 << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y;
        length_sum += (point2 - point1).norm();
        for (size_t j = i; 0 < j; --j)
        {
            if (target_length < length_sum)
            {
                insert_target_point_idx = j + 1;
                break;
            }
            point1 << output.points.at(j).point.pose.position.x, output.points.at(j).point.pose.position.y;
            point2 << output.points.at(j - 1).point.pose.position.x, output.points.at(j - 1).point.pose.position.y;
            length_sum += (point2 - point1).norm();
        }

        // create target point
        Eigen::Vector2d target_point;
        autoware_planning_msgs::PathPointWithLaneId target_point_with_lane_id;
        getBackwordPointFromBasePoint(point2,
                                      point1,
                                      point2,
                                      length_sum - target_length,
                                      target_point);
        target_point_with_lane_id = output.points.at(insert_target_point_idx - 1);
        target_point_with_lane_id.point.pose.position.x = target_point.x();
        target_point_with_lane_id.point.pose.position.y = target_point.y();
        target_point_with_lane_id.point.twist.linear.x = velocity;

        // insert target point
        output.points.insert(output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

        // insert 0 velocity after target point
        for (size_t j = insert_target_point_idx; j < output.points.size(); ++j)
            output.points.at(j).point.twist.linear.x = std::min(velocity, output.points.at(j).point.twist.linear.x);
        // -- debug code --
        if (velocity == 0.0)
            manager_ptr_->debuger.pushStopPose(target_point_with_lane_id.point.pose);
        // ----------------
        return true;
    }
    return false;
}

bool TrafficLightModule::getBackwordPointFromBasePoint(const Eigen::Vector2d &line_point1,
                                                       const Eigen::Vector2d &line_point2,
                                                       const Eigen::Vector2d &base_point,
                                                       const double backward_length,
                                                       Eigen::Vector2d &output_point)
{
    Eigen::Vector2d line_vec = line_point2 - line_point1;
    Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
    output_point = base_point + backward_vec;
    return true;
}

TrafficLightModuleManager::TrafficLightModuleManager() : nh_(""), pnh_("~")
{
}

bool TrafficLightModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId &input,
                                               std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr)
{
    geometry_msgs::PoseStamped self_pose;
    if (!getCurrentSelfPose(self_pose))
        return false;
    lanelet::LaneletMapConstPtr lanelet_map_ptr;
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
        return false;

    for (const auto &point : input.points)
    {
        for (const auto &lane_id : point.lane_ids)
        {
            std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems =
                (lanelet_map_ptr->laneletLayer.get(lane_id)).regulatoryElementsAs<const lanelet::TrafficLight>();
            for (const auto &tl_reg_elem : tl_reg_elems)
            {
                if (!isRunning(*tl_reg_elem))
                {
                    v_module_ptr.push_back(std::make_shared<TrafficLightModule>(this, tl_reg_elem, lane_id));
                }
            }
        }
    }

    return true;
}

bool TrafficLightModuleManager::run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output)
{
    autoware_planning_msgs::PathWithLaneId input_path = input;
    for (size_t i = 0; i < scene_modules_ptr_.size(); ++i)
    {
        autoware_planning_msgs::PathWithLaneId output_path;
        if (scene_modules_ptr_.at(i)->run(input_path, output_path))
            input_path = output_path;
    }
    debuger.publish();
    output = input_path;
    return true;
}

bool TrafficLightModuleManager::isRunning(const lanelet::TrafficLight &traffic_light)
{
    lanelet::ConstLineString3d tl_stop_line;
    lanelet::Optional<lanelet::ConstLineString3d> tl_stopline_opt = traffic_light.stopLine();
    if (!!tl_stopline_opt)
        tl_stop_line = tl_stopline_opt.get();
    else
    {
        ROS_ERROR("cannot get traffic light stop line. This is dangerous. f**k lanelet2. (line: %d)", __LINE__);
        return true;
    }
    // lanelet::ConstLineString3d tl_stop_line = *(traffic_light.stopLine());
    if (task_id_direct_map_.count(tl_stop_line) == 0)
        return false;
    return true;
}

bool TrafficLightModuleManager::registerTask(const lanelet::TrafficLight &traffic_light, const boost::uuids::uuid &uuid)
{
    ROS_INFO("Registered Traffic Light Task");
    lanelet::ConstLineString3d tl_stop_line;
    lanelet::Optional<lanelet::ConstLineString3d> tl_stopline_opt = traffic_light.stopLine();
    if (!!tl_stopline_opt)
        tl_stop_line = tl_stopline_opt.get();
    else
    {
        ROS_ERROR("cannot get traffic light stop line. This is dangerous. f**k lanelet2. (line: %d)", __LINE__);
        return false;
    }
    // const lanelet::ConstLineString3d tl_stop_line = *(traffic_light.stopLine());
    task_id_direct_map_.emplace(tl_stop_line, boost::lexical_cast<std::string>(uuid));
    task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), tl_stop_line);
    return true;
}
bool TrafficLightModuleManager::unregisterTask(const boost::uuids::uuid &uuid)
{
    ROS_INFO("Unregistered Traffic Light Task");
    task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
    task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
    return true;
}

} // namespace behavior_planning