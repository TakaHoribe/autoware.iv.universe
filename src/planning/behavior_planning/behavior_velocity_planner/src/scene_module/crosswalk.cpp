#include <scene_module/crosswalk/crosswalk.hpp>
#include <behavior_velocity_planner/api.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <utilization/geometry/polygon2d.hpp>

namespace behavior_planning
{
CrosswalkModule::CrosswalkModule(CrosswalkModuleManager *manager_ptr,
                                 const lanelet::ConstLanelet &crosswalk,
                                 const int lane_id)
    : manager_ptr_(manager_ptr),
      state_(State::APPROARCH),
      crosswalk_(crosswalk),
      lane_id_(lane_id),
      task_id_(boost::uuids::random_generator()())
{
    if (manager_ptr_ != nullptr)
        manager_ptr_->registerTask(crosswalk_, task_id_);
}

bool CrosswalkModule::run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output)
{
    output = input;

    lanelet::CompoundPolygon3d lanelet_polygon = crosswalk_.polygon3d();
    std::vector<Eigen::Vector2d> points;
    for (const auto &lanelet_point : lanelet_polygon)
    {
        Eigen::Vector2d point;
        point << lanelet_point.x(), lanelet_point.y();
        points.push_back(point);
    }
    Polygon2d polygon(points);

    // check person in polygon
    std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> objects_ptr = std::make_shared<autoware_perception_msgs::DynamicObjectArray>();
    if (!getDynemicObjects(objects_ptr))
        return false;
        bool pedestrian_found = false;
    for (size_t i = 0; i < objects_ptr->objects.size(); ++i)
    {
        if (objects_ptr->objects.at(i).semantic.type == autoware_perception_msgs::Semantic::PEDESTRIAN)
        {
            Eigen::Vector2d point;
            point << objects_ptr->objects.at(i).state.pose_covariance.pose.position.x, objects_ptr->objects.at(i).state.pose_covariance.pose.position.y;
            if (polygon.isInPolygon(point))
                pedestrian_found = true;
        }
    }

    if (!pedestrian_found)
        return true;

    // insert stop point
    for (size_t i = 0; i < output.points.size() - 1; ++i)
    {
        std::vector<Eigen::Vector2d> line;
        Eigen::Vector2d point;
        point << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y;
        line.push_back(point);
        point << output.points.at(i + 1).point.pose.position.x, output.points.at(i + 1).point.pose.position.y;
        line.push_back(point);
        std::vector<Eigen::Vector2d> collision_points;
        if (polygon.getCollisionPoints(line, collision_points))
        {
            for (size_t j = i + 1; j < output.points.size(); ++j)
                output.points.at(j).point.twist.linear.x = 0;
            // -- debug code --
            for (const auto &collision_point : collision_points)
            {
                manager_ptr_->debuger.pushCollisionPoint(collision_point);
            }
            // ----------------
            // -- debug code --
            std::vector<Eigen::Vector3d> line3d;
            Eigen::Vector3d point3d;
            point3d << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y, output.points.at(i).point.pose.position.z;
            line3d.push_back(point3d);
            point3d << output.points.at(i + 1).point.pose.position.x, output.points.at(i + 1).point.pose.position.y, output.points.at(i + 1).point.pose.position.z;
            line3d.push_back(point3d);
            manager_ptr_->debuger.pushCollisionLine(line3d);
            // ----------------
            break;
        }
    }
    return true;
}

bool CrosswalkModule::endOfLife(const autoware_planning_msgs::PathWithLaneId &input)
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
    if (is_end_of_life)
        if (manager_ptr_ != nullptr)
            manager_ptr_->unregisterTask(task_id_);
    return is_end_of_life;
}

CrosswalkModuleManager::CrosswalkModuleManager()
{
    lanelet::LaneletMapConstPtr lanelet_map_ptr;
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
    while (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr) && ros::ok())
    {
        ros::spinOnce();
    }
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
        lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    lanelet::traffic_rules::TrafficRulesPtr pedestrian_rules =
        lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
    lanelet::routing::RoutingGraphConstPtr vehicle_graph = lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *traffic_rules);
    lanelet::routing::RoutingGraphConstPtr pedestrian_graph = lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *pedestrian_rules);
    lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
    overall_graphs_ptr_ = std::make_shared<lanelet::routing::RoutingGraphContainer>(overall_graphs);
}

bool CrosswalkModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId &input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr)
{
    geometry_msgs::PoseStamped self_pose;
    if (!getCurrentSelfPose(self_pose))
        return false;
    lanelet::LaneletMapConstPtr lanelet_map_ptr;
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
    {
        return false;
    }
    if (overall_graphs_ptr_ == nullptr)
        return false;

    for (size_t i = 0; i < input.points.size(); ++i)
    {
        for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j)
        {
            lanelet::ConstLanelet road_lanelet = lanelet_map_ptr->laneletLayer.get(input.points.at(i).lane_ids.at(j));
            std::vector<lanelet::ConstLanelet> crosswalks = overall_graphs_ptr_->conflictingInGraph(road_lanelet, 1);
            for (const auto &crosswalk : crosswalks)
            {
                if (!isRunning(crosswalk))
                    v_module_ptr.push_back(std::make_shared<CrosswalkModule>(this, crosswalk, input.points.at(i).lane_ids.at(j)));
                // -- debug code --
                std::vector<Eigen::Vector3d> points;
                for (const auto &lanelet_point : crosswalk.polygon3d())
                {
                    Eigen::Vector3d point;
                    point << lanelet_point.x(), lanelet_point.y(), lanelet_point.z();
                    points.push_back(point);
                }
                debuger.pushCrosswalkPolygon(points);
                // ----------------
            }
        }
    }
    return true;
}

bool CrosswalkModuleManager::run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output)
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

bool CrosswalkModuleManager::isRunning(const lanelet::ConstLanelet &crosswalk)
{
    if (task_id_direct_map_.count(crosswalk) == 0)
        return false;
    return true;
}

bool CrosswalkModuleManager::registerTask(const lanelet::ConstLanelet &crosswalk, const boost::uuids::uuid &uuid)
{
    ROS_INFO("Registered Crosswalk Task");
    task_id_direct_map_.emplace(crosswalk, boost::lexical_cast<std::string>(uuid));
    task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), crosswalk);
    return true;
}

bool CrosswalkModuleManager::unregisterTask(const boost::uuids::uuid &uuid)
{
    ROS_INFO("Unregistered Crosswalk Task");
    task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
    task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
    return true;
}

CrosswalkDebugMarkersManager::CrosswalkDebugMarkersManager() : nh_(), pnh_("~")
{
    debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/crosswalk", 1);
}

void CrosswalkDebugMarkersManager::pushCollisionLine(const std::vector<Eigen::Vector3d> &line)
{
    collision_lines_.push_back(std::vector<Eigen::Vector3d>(line));
}

void CrosswalkDebugMarkersManager::pushCollisionLine(const std::vector<Eigen::Vector2d> &line)
{
    std::vector<Eigen::Vector3d> line3d;
    for (const auto &point : line)
    {
        Eigen::Vector3d point3d;
        point3d << point.x(), point.y(), 0.0;
        line3d.push_back(point3d);
    }
    pushCollisionLine(line3d);
}

void CrosswalkDebugMarkersManager::pushCollisionPoint(const Eigen::Vector3d &point)
{
    collision_points_.push_back(Eigen::Vector3d(point));
}

void CrosswalkDebugMarkersManager::pushCollisionPoint(const Eigen::Vector2d &point)
{
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0.0;
    pushCollisionPoint(point3d);
}

void CrosswalkDebugMarkersManager::pushCrosswalkPolygon(const std::vector<Eigen::Vector3d> &polygon)
{
    crosswalk_polygons_.push_back(std::vector<Eigen::Vector3d>(polygon));
}

void CrosswalkDebugMarkersManager::pushCrosswalkPolygon(const std::vector<Eigen::Vector2d> &polygon)
{
    std::vector<Eigen::Vector3d> polygon3d;
    for (const auto &point : polygon)
    {
        Eigen::Vector3d point3d;
        point3d << point.x(), point.y(), 0;
        polygon3d.push_back(point3d);
    }
    pushCrosswalkPolygon(polygon3d);
}

void CrosswalkDebugMarkersManager::publish()
{
    visualization_msgs::MarkerArray msg;
    ros::Time current_time = ros::Time::now();
    // Crosswalk polygons
    for (size_t i = 0; i < crosswalk_polygons_.size(); ++i)
    {
        std::vector<Eigen::Vector3d> polygon = crosswalk_polygons_.at(i);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;

        marker.ns = "crosswalk polygon line";
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < polygon.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = polygon.at(j).x();
            point.y = polygon.at(j).y();
            point.z = polygon.at(j).z();
            marker.points.push_back(point);
        }
        marker.points.push_back(marker.points.front());
        msg.markers.push_back(marker);

        marker.ns = "crosswalk polygon point";
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < polygon.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = polygon.at(j).x();
            point.y = polygon.at(j).y();
            point.z = polygon.at(j).z();
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }

    // Collision line
    for (size_t i = 0; i < collision_lines_.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;
        marker.ns = "collision line";
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < collision_lines_.at(i).size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = collision_lines_.at(i).at(j).x();
            point.y = collision_lines_.at(i).at(j).y();
            point.z = collision_lines_.at(i).at(j).z();
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }

    // Collision point
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;
        marker.ns = "collision point";
        marker.id = 0;
        marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < collision_points_.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = collision_points_.at(j).x();
            point.y = collision_points_.at(j).y();
            point.z = collision_points_.at(j).z();
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }

    debug_viz_pub_.publish(msg);
    collision_points_.clear();
    collision_lines_.clear();
    crosswalk_polygons_.clear();
    return;
}

} // namespace behavior_planning