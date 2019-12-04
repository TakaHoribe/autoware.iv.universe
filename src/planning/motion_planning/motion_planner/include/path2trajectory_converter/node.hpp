#ifndef PATH2TRAJECTORY_CONVERTER_H
#define PATH2TRAJECTORY_CONVERTER_H
#include "motion_base_planner/node.hpp"


namespace motion_planner
{
class Path2Trajectory : public  BasePlannerNode
{
private:
  void callback(const autoware_planning_msgs::Path &input_path_msg, 
                autoware_planning_msgs::Trajectory &output_trajectory_msg) override;

  
public:
   Path2Trajectory();
  ~Path2Trajectory();
};
}

#endif