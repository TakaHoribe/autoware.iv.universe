#include <scene_module/right_turn/right_turn.hpp>
#include <behavior_velocity_planner/api.hpp>

#include "util/util.h"

// clang-format off
#define DEBUG_INFO(...) { if (show_debug_info_) { ROS_INFO(__VA_ARGS__); }}

// clang-format on
namespace behavior_planning
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;

/*
 * ========================= Right Turn Module =========================
 */
RightTurnModule::RightTurnModule(const int lane_id, RightTurnModuleManager *right_turn_module_manager)
    : assigned_lane_id_(lane_id), right_turn_module_manager_(right_turn_module_manager)
{
    judge_line_dist_ = 3.0;                      // [m]
    approaching_speed_to_stopline_ = 10.0 / 3.6; // 10[km/h]
    state_machine_.setMarginTime(2.0);           // [sec]
    path_expand_width_ = 2.0;
    show_debug_info_ = false;
};

bool RightTurnModule::run(const autoware_planning_msgs::PathWithLaneId &input,
                          autoware_planning_msgs::PathWithLaneId &output)
{
    output = input;

    right_turn_module_manager_->debugger_.publishPath(output, "path_raw", 0.0, 1.0, 1.0);

    /* set stop-line and stop-judgement-line */
    if (!setStopLineIdx(judge_line_dist_, output, stop_line_idx_, judge_line_idx_))
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "[RightTurnModule::run] setStopLineIdx fail");
        return false;
    }
    right_turn_module_manager_->debugger_.publishPose(output.points.at(stop_line_idx_).point.pose, "stop_point_pose", 1.0, 1.0, 0.0);
    right_turn_module_manager_->debugger_.publishPose(output.points.at(judge_line_idx_).point.pose, "judge_point_pose", 1.0, 1.0, 0.5);
    right_turn_module_manager_->debugger_.publishPath(output, "path_with_judgeline", 0.0, 0.5, 1.0);


    /* set approaching speed to stop-line */
    setVelocityFrom(judge_line_idx_, approaching_speed_to_stopline_, output);

    /* get current pose */
    geometry_msgs::PoseStamped current_pose;
    if (!getCurrentSelfPose(current_pose))
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "[RightTurnModule::run] getCurrentSelfPose fail");
        return false;
    }

    /* check if the current_pose is ahead from judgement line */
    int closest = -1;
    if (!planning_utils::calcClosestIndex(output, current_pose.pose, closest))
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "[RightTurnModule::run] calcClosestIndex fail");
        return false;
    }

    if (state_machine_.getState() == State::GO)
    {
        geometry_msgs::Pose p = planning_utils::transformOrigin2D(current_pose.pose, output.points.at(judge_line_idx_).point.pose);
        if (p.position.x > 0.0) // current_pose is ahead of judge_line
        {
            DEBUG_INFO("[RightTurnModule::run] no plan needed. skip collision check.");
            return true; // no plan needed.
        }

    }

    /* get lanelet map */
    lanelet::LaneletMapConstPtr lanelet_map_ptr;              // objects info
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr; // route info
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "[RightTurnModuleManager::run()] cannot get lanelet map");
        return false;
    }

    /* get detection area */
    lanelet::ConstLanelet assigned_lanelet = lanelet_map_ptr->laneletLayer.get(assigned_lane_id_); // current assigned lanelets
    std::vector<lanelet::ConstLanelet> objective_lanelets = lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
    right_turn_module_manager_->debugger_.publishLaneletsArea(objective_lanelets, "right_turn_detection_lanelets");
    DEBUG_INFO("[RightTurnModuleManager::run()] assigned_lane_id_ = %d, objective_lanelets.size() = %lu", assigned_lane_id_, objective_lanelets.size());
    if (objective_lanelets.empty())
    {
        DEBUG_INFO("[RightTurnModule::run]: detection area number is zero. skip computation.");
        return true;
    }

    /* get dynamic object */
    std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr = std::make_shared<autoware_perception_msgs::DynamicObjectArray>();
    if (!getDynemicObjects(objects_ptr))
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "[RightTurnModuleManager::run()] cannot get dynamic object");
        return false;
    }

    /* calculate dynamic collision around detection area */
    bool is_collision = false;
    if (!checkCollision(output, objective_lanelets, objects_ptr, path_expand_width_, is_collision))
    {
        return false;
    }

    if (is_collision)
    {
        state_machine_.setStateWithMarginTime(State::STOP);
    }
    else
    {
        state_machine_.setStateWithMarginTime(State::GO);
    }

    /* set stop speed */
    if (state_machine_.getState() == State::STOP)
    {
        const double stop_vel = 0.0;
        setVelocityFrom(stop_line_idx_, stop_vel, output);
    }

    return true;
}

bool RightTurnModule::endOfLife(const autoware_planning_msgs::PathWithLaneId &input)
{

    /* search if the assigned lane_id is still exists */
    bool is_assigned_lane_id_found = false;
    for (const auto &point : input.points)
    {
        for (const auto &id : point.lane_ids)
        {
            if (assigned_lane_id_ == id)
            {
                is_assigned_lane_id_found = true;
                break;
            }
        }
        if (is_assigned_lane_id_found == true)
        {
            break;
        }
    }

    bool is_end_of_life = !is_assigned_lane_id_found;

    if (is_end_of_life)
    {
        right_turn_module_manager_->unregisterTask(assigned_lane_id_);
    }

    return is_end_of_life;
}

bool RightTurnModule::setStopLineIdx(const double judge_line_dist, autoware_planning_msgs::PathWithLaneId &path,
                                     int &stop_line_idx, int &judge_line_idx)
{

    // TEMP: return first assigned_lane_id point's index
    stop_line_idx = -1;
    for (size_t i = 0; i < path.points.size(); ++i)
    {
        for (const auto &id : path.points.at(i).lane_ids)
        {
            if (id == assigned_lane_id_)
            {
                stop_line_idx = i;
            }
            if (stop_line_idx != -1)
                break;
        }
        if (stop_line_idx != -1)
            break;
    }

    if (stop_line_idx == -1)
    {
        ROS_ERROR("[RightTurnModule::setStopLineIdx]: cannot set the stop line. something wrong. please check code. ");
        return false; // cannot find stop line.
    }

    // TEMP: should use interpolation (points distance may be very long)
    double curr_dist = 0.0;
    double prev_dist = curr_dist;
    bool enable_interpolation = true;
    judge_line_idx = -1;
    for (size_t i = stop_line_idx; i > 0; --i)
    {
        const geometry_msgs::Point p0 = path.points.at(i).point.pose.position;
        const geometry_msgs::Point p1 = path.points.at(i - 1).point.pose.position;
        // const double dx = p0.x - p1.x;
        // const double dy = p0.y - p1.y;
        // curr_dist += std::sqrt(dx * dx + dy * dy);
        curr_dist += planning_utils::calcDist2d(p0, p1);
        // printf("i = %d, dx = %f, dy = %f, curr_dist = %f\n", i, dx, dy, curr_dist);
        if (curr_dist > judge_line_dist)
        {
            if (enable_interpolation) // TEMP implementation
            {
                const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
                const double w_p0 = (curr_dist - judge_line_dist) / dl;
                const double w_p1 = (judge_line_dist - prev_dist) / dl;
                autoware_planning_msgs::PathPointWithLaneId p = path.points.at(i);
                p.point.pose.position.x = w_p0 * p0.x + w_p1 * p1.x;
                p.point.pose.position.y = w_p0 * p0.y + w_p1 * p1.y;
                p.point.pose.position.z = w_p0 * p0.z + w_p1 * p1.z;
                auto itr = path.points.begin();
                itr += i;
                path.points.insert(itr, p);
                judge_line_idx = i;
                break;
            }
            else
            {
                judge_line_idx = i - 1;
                break;
            }
        }
        prev_dist = curr_dist;
    }
    if (judge_line_idx == -1)
    {
        ROS_ERROR("[RightTurnModule::setStopLineIdx]: cannot set the stop judgement line. path is too short.");
        judge_line_idx = 0;
    }
    return true;
}

bool RightTurnModule::setVelocityFrom(const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId &input)
{
    for (size_t i = idx; i < input.points.size(); ++i)
    {
        input.points.at(i).point.twist.linear.x = vel;
    }
}

Polygon RightTurnModule::convertToBoostGeometryPolygon(const lanelet::ConstLanelet &lanelet)
{
    Polygon polygon;
    lanelet::CompoundPolygon3d lanelet_polygon = lanelet.polygon3d();
    for (const auto &lanelet_point : lanelet_polygon)
    {
        polygon.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
    }
    polygon.outer().push_back(polygon.outer().front());
    return polygon;
}

bool RightTurnModule::checkCollision(const autoware_planning_msgs::PathWithLaneId &path,
                                     const std::vector<lanelet::ConstLanelet> &objective_lanelets,
                                     const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr,
                                     const double path_width, bool &is_collision)
{
    /* generates side edge line */
    autoware_planning_msgs::PathWithLaneId path_r; // right side edge line
    autoware_planning_msgs::PathWithLaneId path_l; // left side edge line
    generateEdgeLine(path, path_width, path_r, path_l);


    /* check collision for each objects and lanelets area */
    is_collision = false;
    for (size_t i = 0; i < objective_lanelets.size(); ++i) // for each objective lanelets
    {
        Polygon polygon = convertToBoostGeometryPolygon(objective_lanelets.at(i));

        for (size_t j = 0; j < objects_ptr->objects.size(); ++j) // for each dynamic objects
        {
            Point point(objects_ptr->objects.at(j).state.pose_covariance.pose.position.x, objects_ptr->objects.at(j).state.pose_covariance.pose.position.y);
            if (bg::within(point, polygon)) // if the dynamic object is in the lanelet polygon, check collision
            {
                // ROS_INFO("lanelet_id: %lu, object_no: %lu, INSIDE POLYGON", i, j);
                if(checkPathCollision(path_r, objects_ptr->objects.at(j)) || checkPathCollision(path_l, objects_ptr->objects.at(j)))
                {
                    is_collision = true;
                }
            }
            else
            {
                // ROS_INFO("lanelet_id: %lu, object_no: %lu, out of polygon", i, j);
            }

            if (is_collision)
                break;
        }
        if (is_collision)
            break;
    }

    /* for debug */
    right_turn_module_manager_->debugger_.publishPath(path_r, "path_right_edge", 0.5, 0.0, 0.5);
    right_turn_module_manager_->debugger_.publishPath(path_l, "path_left_edge", 0.0, 0.5, 0.5);

    return true;
}

bool RightTurnModule::checkPathCollision(const autoware_planning_msgs::PathWithLaneId &path,
                                         const autoware_perception_msgs::DynamicObject &object)
{
    bool is_collision = false;

    bg::model::linestring<Point> bg_ego_path;
    for (const auto &p : path.points)
    {
        bg_ego_path.push_back(Point{p.point.pose.position.x, p.point.pose.position.y});
    }

    std::vector<bg::model::linestring<Point>> bg_object_path_arr;
    for (size_t i = 0; i < object.state.predicted_paths.size(); ++i)
    {
        bg::model::linestring<Point> bg_object_path;
        for (const auto &p : object.state.predicted_paths.at(i).path)
        {
            bg_object_path.push_back(Point{p.pose.pose.position.x, p.pose.pose.position.y});
        }
        bg_object_path_arr.push_back(bg_object_path);
    }

    for (size_t i = 0; i < object.state.predicted_paths.size(); ++i)
    {
        bool is_intersects = bg::intersects(bg_ego_path, bg_object_path_arr.at(i));
        is_collision = is_collision || is_intersects;
    }

    return is_collision;
}

bool RightTurnModule::generateEdgeLine(const autoware_planning_msgs::PathWithLaneId &path, const double path_width,
                                       autoware_planning_msgs::PathWithLaneId &path_r, autoware_planning_msgs::PathWithLaneId &path_l)
{
    path_r = path;
    path_l = path;
    for (int i = 0; i < path.points.size(); ++i)
    {
        const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
        path_r.points.at(i).point.pose.position.x += path_width * std::sin(yaw);
        path_r.points.at(i).point.pose.position.y -= path_width * std::cos(yaw);
        path_l.points.at(i).point.pose.position.x -= path_width * std::sin(yaw);
        path_l.points.at(i).point.pose.position.y += path_width * std::cos(yaw);
    }
}


void RightTurnModule::StateMachine::setStateWithMarginTime(RightTurnModule::State state)
{
    /* same state request */
    if (state_ == state)
    {
        start_time_ = nullptr; // reset timer
        return;
    }

    /* GO -> STOP */
    if (state == State::STOP)
    {
        state_ = State::STOP;
        start_time_ = nullptr; // reset timer
        return;
    }

    /* STOP -> GO */
    if (state == State::GO)
    {
        if (start_time_ == nullptr)
        {
            start_time_ = std::make_shared<ros::Time>(ros::Time::now());
            return;
        }
        else
        {
            const double duration = (ros::Time::now() - *start_time_).toSec();
            if (duration > margin_time_)
            {
                state_ = State::GO;
                start_time_ = nullptr; // reset timer
                // ROS_INFO("[RightTurnModule::StateMachine::setStateWithMarginTime()]: timer counting... (%3.3f < %3.3f)", duration, margin_time_);
            }
            else
            {
                // ROS_INFO("[RightTurnModule::StateMachine::setStateWithMarginTime()]: state changed. STOP -> GO (%3.3f > %3.3f)", duration, margin_time_);
            }
            return;
        }
    }

    ROS_ERROR("[RightTurnModule::StateMachine::setStateWithMarginTime()] : Unsuitable state. ignore request.");
    return;
}

void RightTurnModule::StateMachine::setState(RightTurnModule::State state)
{
    state_ = state;
}

void RightTurnModule::StateMachine::setMarginTime(const double t)
{
    margin_time_ = t;
}

RightTurnModule::State RightTurnModule::StateMachine::getState()
{
    return state_;
}
/*
 * ========================= Right Turn Module Manager =========================
 */
bool RightTurnModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId &input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr)
{
    /* get self pose */
    geometry_msgs::PoseStamped self_pose;
    if (!getCurrentSelfPose(self_pose))
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "[RightTurnModuleManager::startCondition()] cannot get current self pose");
        return false;
    }

    /* get lanelet map */
    lanelet::LaneletMapConstPtr lanelet_map_ptr;              // objects info
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr; // route info
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
    {
        ROS_WARN_DELAYED_THROTTLE(1.0, "[RightTurnModuleManager::startCondition()] cannot get lanelet map");
        return false;
    }

    /* search right turn tag */
    for (size_t i = 0; i < input.points.size(); ++i)
    {
        for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j)
        {
            const int lane_id = input.points.at(i).lane_ids.at(j);
            lanelet::ConstLanelet lanelet_ij = lanelet_map_ptr->laneletLayer.get(lane_id); // get lanelet layer
            std::string turn_direction = lanelet_ij.attributeOr("turn_direction", "else"); // get turn_direction

            if (turn_direction.compare("right") == 0 && !isRunning(lane_id))
            {
                // right turn tag is found. set module.
                v_module_ptr.push_back(std::make_shared<RightTurnModule>(lane_id, this));
                registerTask(lane_id);
            }
        }
    }

    return true;
}

bool RightTurnModuleManager::isRunning(const int lane_id)
{
    for (const auto &id : registered_lane_ids_)
    {
        if (id == lane_id)
            return true;
    }
    return false;
}

void RightTurnModuleManager::registerTask(const int lane_id)
{
    registered_lane_ids_.push_back(lane_id);
}

void RightTurnModuleManager::unregisterTask(const int lane_id)
{
    const auto itr = std::find(registered_lane_ids_.begin(), registered_lane_ids_.end(), lane_id);
    if (itr == registered_lane_ids_.end())
        ROS_ERROR("[RightTurnModuleManager::unregisterTask()] : cannot remove task (lane_id = %d,"
                  " registered_lane_ids_.size() = %lu)",
                  lane_id, registered_lane_ids_.size());
    registered_lane_ids_.erase(itr);
}

/*
 * ========================= Right Turn Module Debugger =========================
 */
RightTurnModuleDebugger::RightTurnModuleDebugger() : nh_(""), pnh_("~")
{
    debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/right_turn", 1);
}

void RightTurnModuleDebugger::publishLaneletsArea(const std::vector<lanelet::ConstLanelet> &lanelets, const std::string &ns)
{
    ros::Time curr_time = ros::Time::now();
    visualization_msgs::MarkerArray msg;

    for (size_t i = 0; i < lanelets.size(); ++i)
    {
        lanelet::CompoundPolygon3d lanelet_i_polygon = lanelets.at(i).polygon3d();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = curr_time;

        marker.ns = ns + "_" + std::to_string(i);
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < lanelet_i_polygon.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = lanelet_i_polygon[j].x();
            point.y = lanelet_i_polygon[j].y();
            point.z = lanelet_i_polygon[j].z();
            marker.points.push_back(point);
        }
        marker.points.push_back(marker.points.front());
        msg.markers.push_back(marker);
    }
    debug_viz_pub_.publish(msg);
}

void RightTurnModuleDebugger::publishPath(const autoware_planning_msgs::PathWithLaneId &path, const std::string &ns, double r, double g, double b)
{
    ros::Time curr_time = ros::Time::now();
    visualization_msgs::MarkerArray msg;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = curr_time;
    marker.ns = ns;
    
    for (int i = 0; i < path.points.size(); ++i)
    {
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = path.points.at(i).point.pose;
        marker.scale.x = 0.5;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        msg.markers.push_back(marker);
    }

    debug_viz_pub_.publish(msg);
}

void RightTurnModuleDebugger::publishPose(const geometry_msgs::Pose &pose, const std::string &ns, double r, double g, double b)
{
    ros::Time curr_time = ros::Time::now();
    visualization_msgs::MarkerArray msg;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = curr_time;
    marker.ns = ns;

    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    msg.markers.push_back(marker);
    
    debug_viz_pub_.publish(msg);
}


} // namespace behavior_planning