#include <scene_module/right_turn/right_turn.hpp>
#include <behavior_velocity_planner/api.hpp>

namespace behavior_planning
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;

/*
 * ========================= Right Turn Module =========================
 */
RightTurnModule::RightTurnModule(const int lane_id, RightTurnModuleManager *right_turn_module_manager)
    : assigned_lane_id_(lane_id), right_turn_module_manager_(right_turn_module_manager){};

bool RightTurnModule::run(const autoware_planning_msgs::PathWithLaneId &input,
                          autoware_planning_msgs::PathWithLaneId &output)
{
    output = input;

    ROS_INFO("[RightTurnModule] now calculating right turn!!!");

    /* get lanelet map */
    lanelet::LaneletMapConstPtr lanelet_map_ptr;              // objects info
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr; // route info
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
    {
        ROS_WARN_DELAYED_THROTTLE(3.0, "[RightTurnModuleManager::run()] cannot get lanelet map");
        return false;
    }

    /* get detection area */
    lanelet::ConstLanelet assigned_lanelet = lanelet_map_ptr->laneletLayer.get(assigned_lane_id_); // current assigned lanelets
    std::vector<lanelet::ConstLanelet> objective_lanelets = lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
    right_turn_module_manager_->debugger_.publishLaneletsArea(objective_lanelets, "right_turn_detection_lanelets");
    ROS_INFO_DELAYED_THROTTLE(1.0, "[RightTurnModuleManager::run()] assigned_lane_id_ = %d, objective_lanelets.size() = %lu", assigned_lane_id_, objective_lanelets.size());

    /* get dynamic object */
    std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr = std::make_shared<autoware_perception_msgs::DynamicObjectArray>();
    if (!getDynemicObjects(objects_ptr))
    {
        ROS_WARN_DELAYED_THROTTLE(3.0, "[RightTurnModuleManager::run()] cannot get dynamic object");
        return false;
    }

    /* calculate detected objects around detection area */
    bool is_collision = false;
    for (size_t i = 0; i < objective_lanelets.size(); ++i) // for each objective lanelets
    {
        Polygon polygon = convertToBoostGeometryPolygon(objective_lanelets.at(i));

        for (size_t j = 0; j < objects_ptr->objects.size(); ++j) // for each dynamic objects
        {
            Point point(objects_ptr->objects.at(j).state.pose_covariance.pose.position.x, objects_ptr->objects.at(j).state.pose_covariance.pose.position.y);
            if (bg::within(point, polygon)) // if the dynamic object is in the lanelet polygon,
            {
                ROS_WARN("lanelet_id: %lu, object_no: %lu,  INSIDE POLYGON\n", i, j);
                is_collision = checkDynamicCollision(input, objects_ptr->objects.at(j));
            }
            else
            {
                ROS_INFO("lanelet_id: %lu, object_no: %lu, out of polygon\n", i, j);
            }

            if (is_collision)
                break;
        }
        if (is_collision)
            break;
    }

    if (is_collision)
    {
        const size_t stop_point_id = 0;
        setStopVelocityFrom(stop_point_id, output);
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

bool RightTurnModule::setStopVelocityFrom(const size_t stop_point_id, autoware_planning_msgs::PathWithLaneId &input)
{
    for (size_t i = stop_point_id; i < input.points.size(); ++i)
    {
        input.points.at(i).point.twist.linear.x = 0.0;
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

bool RightTurnModule::checkDynamicCollision(const autoware_planning_msgs::PathWithLaneId &path, const autoware_perception_msgs::DynamicObject &object)
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
        ROS_INFO("[RightTurnModule::checkDynamicCollision()]: predicted path no.%d : is_intersects = %d", i, is_intersects);
        is_collision = is_collision || is_intersects;
    }

    return is_collision;
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
        ROS_WARN_DELAYED_THROTTLE(3.0, "[RightTurnModuleManager::startCondition()] cannot get current self pose");
        return false;
    }

    /* get lanelet map */
    lanelet::LaneletMapConstPtr lanelet_map_ptr;              // objects info
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr; // route info
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
    {
        ROS_WARN_DELAYED_THROTTLE(3.0, "[RightTurnModuleManager::startCondition()] cannot get lanelet map");
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
        ROS_ERROR("[RightTurnModuleManager::unregisterTask()] : cannot remove task (lane_id = %d, registered_lane_ids_.size() = %lu)", lane_id, registered_lane_ids_.size());
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

} // namespace behavior_planning