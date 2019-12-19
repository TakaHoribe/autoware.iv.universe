#include <goal_path_refiner/goal_path_refiner.hpp>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace behavior_planning
{
GoalPathRefiner::GoalPathRefiner(){};
GoalPathRefiner::GoalPathRefiner(const double search_radius_range,
                                 const double search_rad_range,
                                 const autoware_planning_msgs::PathWithLaneId &path,
                                 const geometry_msgs::Pose &goal) : radius_threshold_(search_radius_range),
                                                                    rad_threshold_(search_rad_range)
{
    path_with_lane_id_ptr_ = std::make_shared<autoware_planning_msgs::PathWithLaneId>(path);
    goal_ptr_ = std::make_shared<geometry_msgs::Pose>(goal);
}

bool GoalPathRefiner::getRefinedPath(autoware_planning_msgs::PathWithLaneId &output)
{
    if (path_with_lane_id_ptr_ == nullptr || goal_ptr_ == nullptr)
        return false;
    return getRefinedPath(radius_threshold_, rad_threshold_, *path_with_lane_id_ptr_, *goal_ptr_, output);
}

bool GoalPathRefiner::getRefinedPath(const double search_radius_range, const double search_rad_range, const autoware_planning_msgs::PathWithLaneId &input,
                                     const geometry_msgs::Pose &goal,
                                     autoware_planning_msgs::PathWithLaneId &output)
{
    size_t min_dist_index;
    double min_dist;
    double goal_z;
    {
        bool found = false;
        for (size_t i = 0; i < input.points.size(); ++i)
        {
            const double x = input.points.at(i).point.pose.position.x - goal.position.x;
            const double y = input.points.at(i).point.pose.position.y - goal.position.y;
            const double z = input.points.at(i).point.pose.position.z - goal.position.z;
            const double dist = sqrt(x * x + y * y);
            if ((dist < search_radius_range) && (dist < min_dist || !found /*init*/))
            {
                min_dist_index = i;
                min_dist = dist;
                found = true;
            }
        }
        if (!found)
            return false;
    }

    size_t min_dist_out_of_range_index;
    {
        bool found = false;
        for (size_t i = min_dist_index; 0 <= i; --i)
        {
            const double x = input.points.at(i).point.pose.position.x - goal.position.x;
            const double y = input.points.at(i).point.pose.position.y - goal.position.y;
            const double z = input.points.at(i).point.pose.position.z - goal.position.z;
            goal_z = input.points.at(i).point.pose.position.z;
            const double dist = sqrt(x * x + y * y);
            if (search_radius_range < dist)
            {
                min_dist_out_of_range_index = i;
                found = true;
                break;
            }
        }
        if (!found)
            return false;
    }

    autoware_planning_msgs::PathPointWithLaneId refined_goal;
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z = goal_z;
    refined_goal.point.twist.linear.x = 0.0;

    autoware_planning_msgs::PathPointWithLaneId pre_refined_goal;
    double roll, pitch, yaw;
    pre_refined_goal.point.pose = goal;
    tf2::Quaternion tf2_quaternion(goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
    tf2::Matrix3x3 tf2_matrix(tf2_quaternion);
    tf2_matrix.getRPY(roll, pitch, yaw);
    pre_refined_goal.point.pose.position.x -= std::cos(yaw);
    pre_refined_goal.point.pose.position.y -= std::sin(yaw);
    pre_refined_goal.point.pose.position.z = goal_z;
    pre_refined_goal.point.twist.linear.x = 1.0; // 3.6kmph

    for (size_t i = 0; i <= min_dist_out_of_range_index; ++i)
    {
        output.points.push_back(input.points.at(i));
    }
    output.points.push_back(pre_refined_goal);
    output.points.push_back(refined_goal);
    return true;
}

} // namespace behavior_planning