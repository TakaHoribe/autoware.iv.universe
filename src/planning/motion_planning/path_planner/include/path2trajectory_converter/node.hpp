#ifndef PATH2TRAJECTORY_CONVERTER_H
#define PATH2TRAJECTORY_CONVERTER_H
#include "path_base_planner/node.hpp"


namespace path_planner
{
class Path2Trajectory : public  BasePlannerNode
{
private:
  void callback(const autoware_planning_msgs::Path &input_path_msg, 
                autoware_planning_msgs::Trajectory &output_trajectory_msg) override;
  
  void msgConversionFromPath2Trajectory(
    const autoware_planning_msgs::Path& path,
    autoware_planning_msgs::Trajectory& traj);

  
public:
   Path2Trajectory();
  ~Path2Trajectory();
};
}

#endif