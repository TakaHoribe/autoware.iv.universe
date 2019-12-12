#include <utilization/path_utilization.hpp>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

namespace behavior_planning
{
void interporatePath(const autoware_planning_msgs::Path &path, const double length, autoware_planning_msgs::Path &interporated_path)
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> v;
    if (50 < path.points.size())
        ROS_WARN("because path size is too large, calculation cost is high. size is %d.", (int)path.points.size());
    for (const auto &path_point : path.points)
    {
        x.push_back(path_point.pose.position.x);
        y.push_back(path_point.pose.position.y);
        z.push_back(path_point.pose.position.z);
        v.push_back(path_point.twist.linear.x);
    }
    std::shared_ptr<Spline4D> spline_ptr;
    spline_ptr = std::make_shared<Spline4D>(x, y, z, v);
    // std::cout<<"st:"<<spline_ptr_->s.back() << std::endl;
    // std::cout<<"point size:"<<path.points.size() << std::endl;
    double s_t;
    size_t checkpoint_idx = 0;
    int reference_velocity_idx = 0;
    double reference_velocity;
    const double interporation_interval = 1.0;
    for (s_t = interporation_interval; s_t < std::min(length, spline_ptr->s.back()); s_t += interporation_interval)
    {
        while (reference_velocity_idx < spline_ptr->s.size() && spline_ptr->s.at(reference_velocity_idx) < s_t)
        {
            ++reference_velocity_idx;
        }
        reference_velocity = spline_ptr->calc_trajectory_point(spline_ptr->s.at(std::max(0, reference_velocity_idx - 1)))[3];

        // insert check point before interporated point
        while (checkpoint_idx < spline_ptr->s.size() && spline_ptr->s.at(checkpoint_idx) < s_t)
        {
            autoware_planning_msgs::PathPoint path_point;
            std::array<double, 4> state = spline_ptr->calc_trajectory_point(spline_ptr->s.at(checkpoint_idx));
            path_point.pose.position.x = state[0];
            path_point.pose.position.y = state[1];
            path_point.pose.position.z = state[2];
            path_point.twist.linear.x = state[3];
            const double yaw = spline_ptr->calc_yaw(s_t);
            tf2::Quaternion tf2_quaternion;
            tf2_quaternion.setRPY(0, 0, yaw);
            path_point.pose.orientation = tf2::toMsg(tf2_quaternion);
            interporated_path.points.push_back(path_point);
            ++checkpoint_idx;
        }
        autoware_planning_msgs::PathPoint path_point;
        std::array<double, 4> state = spline_ptr->calc_trajectory_point(s_t);
        path_point.pose.position.x = state[0];
        path_point.pose.position.y = state[1];
        path_point.pose.position.z = state[2];
        path_point.twist.linear.x = reference_velocity;
        const double yaw = spline_ptr->calc_yaw(s_t);
        tf2::Quaternion tf2_quaternion;
        tf2_quaternion.setRPY(0, 0, yaw);
        path_point.pose.orientation = tf2::toMsg(tf2_quaternion);

        interporated_path.points.push_back(path_point);

    }
    if (spline_ptr->s.back() <= s_t)
        interporated_path.points.push_back(path.points.back());
}


void filterLitterPathPoint(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path)
{
    const double epsilon = 0.01;
    size_t latest_id = 0;
    for (size_t i = 0; i < path.points.size(); ++i)
    {
        double dist;
        if (i != 0)
        {
            const double x = path.points.at(i).pose.position.x - path.points.at(latest_id).pose.position.x;
            const double y = path.points.at(i).pose.position.y - path.points.at(latest_id).pose.position.y;
            dist = std::sqrt(x * x + y * y);
        }
        if (epsilon < dist || i == 0 /*init*/)
        {
            latest_id = i;
            filtered_path.points.push_back(path.points.at(latest_id));
        } else {
            filtered_path.points.back().twist.linear.x = std::min(filtered_path.points.back().twist.linear.x,
                                                                  path.points.at(i).twist.linear.x);
        }
    }
}
void filterStopPathPoint(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path){
    filtered_path = path;
    bool found_stop = false;
    for (size_t i = 0; i < filtered_path.points.size(); ++i)
    {
        if (std::fabs(filtered_path.points.at(i).twist.linear.x) < 0.01)
            found_stop = true;
        if (found_stop)
            filtered_path.points.at(i).twist.linear.x = 0.0;
    }
}

} // namespace behavior_planning
