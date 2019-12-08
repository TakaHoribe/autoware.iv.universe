#include <scene_module/momentary_stop/momentary_stop.hpp>
#include <behavior_velocity_planner/api.hpp>

namespace behavior_planning
{
MomentaryStopModule::MomentaryStopModule(const lanelet::ConstLineString3d &stop_line) : state_(State::APPROARCH), stop_line_(stop_line),
                                                                                        task_id_(boost::uuids::random_generator()())
{
    MomentaryStopCondition::registerTask(stop_line, task_id_);
};

bool MomentaryStopModule::run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output)
{
    output = input;
    const double stop_point_x = (stop_line_[0].x() + stop_line_[1].x()) / 2.0;
    const double stop_point_y = (stop_line_[0].y() + stop_line_[1].y()) / 2.0;
    if (state_ == State::APPROARCH)
    {
        double min_dist;
        size_t min_dist_index;
        for (size_t i = 0; i < output.points.size(); ++i)
        {
            const double x = output.points.at(i).point.pose.position.x - stop_point_x;
            const double y = output.points.at(i).point.pose.position.y - stop_point_y;
            const double dist = std::sqrt(x * x + y * y);
            if (dist < min_dist || i == 0 /*init*/)
            {
                min_dist = dist;
                min_dist_index = i;
            }
        }
        for (size_t i = min_dist_index; i < output.points.size(); ++i)
        {
            output.points.at(i).point.twist.linear.x = 0.0;
        }

        geometry_msgs::PoseStamped self_pose;
        if (!getCurrentSelfPose(self_pose))
            return true;
        const double x = stop_point_x - self_pose.pose.position.x;
        const double y = stop_point_y - self_pose.pose.position.y;
        const double dist = std::sqrt(x * x + y * y);
        if (dist < 2.0 && isVehicleStopping())
            state_ = State::STOP;
        return true;
    }
    else if (state_ == State::STOP)
    {
        if (!isVehicleStopping())
            state_ = State::START;
        return true;
    }
}
bool MomentaryStopModule::endOfLife(const autoware_planning_msgs::PathWithLaneId &input)
{
    bool is_end_of_life = false;
    geometry_msgs::PoseStamped self_pose;
    if (!getCurrentSelfPose(self_pose))
        return false;
    const double stop_point_x = (stop_line_[0].x() + stop_line_[1].x()) / 2.0;
    const double stop_point_y = (stop_line_[0].y() + stop_line_[1].y()) / 2.0;
    const double x = stop_point_x - self_pose.pose.position.x;
    const double y = stop_point_y - self_pose.pose.position.y;
    const double dist = std::sqrt(x * x + y * y);
    // if (state_ == State::START && 5.0 < dist)
    //     is_end_of_life = true;

    if (is_end_of_life)
        MomentaryStopCondition::unregisterTask(task_id_);
    return is_end_of_life;
}
bool MomentaryStopCondition::startCondition(const autoware_planning_msgs::PathWithLaneId &input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr)
{
    geometry_msgs::PoseStamped self_pose;
    if (!getCurrentSelfPose(self_pose))
        return false;
    lanelet::LaneletMapConstPtr lanelet_map_ptr;
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
        return false;
    for (size_t i = 0; i < input.points.size(); ++i)
    {
        for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j)
        {
            std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems =
                (lanelet_map_ptr->laneletLayer.get(input.points.at(i).lane_ids.at(j))).regulatoryElementsAs<const lanelet::TrafficSign>();
            if (traffic_sign_reg_elems.empty())
                continue;
            std::shared_ptr<const lanelet::TrafficSign> traffic_sign = traffic_sign_reg_elems.front();
            if (traffic_sign->type() == "stop_sign")
            {
                lanelet::ConstLineStrings3d traffic_sign_stoplines = traffic_sign->refLines();
                for (const auto &traffic_sign_stopline : traffic_sign_stoplines)
                {
                    if (!isRunning(traffic_sign_stopline))
                    {
                        v_module_ptr.push_back(std::make_shared<MomentaryStopModule>(traffic_sign_stopline));
                    }
                }
            }
        }
    }
    return true;
}

bool MomentaryStopCondition::isRunning(const lanelet::ConstLineString3d &stop_line)
{
    if (task_id_direct_map_.count(stop_line) == 0)
        return false;
    return true;
}

bool MomentaryStopCondition::registerTask(const lanelet::ConstLineString3d &stop_line, const boost::uuids::uuid &uuid)
{
    task_id_direct_map_.emplace(stop_line, boost::lexical_cast<std::string>(uuid));
    task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), stop_line);
    return true;
}
bool MomentaryStopCondition::unregisterTask(const boost::uuids::uuid &uuid)
{
    task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
    task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
    return true;
}

std::unordered_map<lanelet::ConstLineString3d, std::string> MomentaryStopCondition::task_id_direct_map_;
std::unordered_map<std::string, lanelet::ConstLineString3d> MomentaryStopCondition::task_id_reverse_map_;

} // namespace behavior_planning