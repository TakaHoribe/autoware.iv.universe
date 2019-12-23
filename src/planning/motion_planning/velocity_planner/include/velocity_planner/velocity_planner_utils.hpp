
#include <iostream>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <boost/shared_ptr.hpp>

#include <autoware_planning_msgs/Trajectory.h>

namespace vpu
{
double square(const double &a);
double calcSquaredDist2d(const geometry_msgs::Point &a, const geometry_msgs::Point &b);
double calcSquaredDist2d(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b);
double calcSquaredDist2d(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b);
double calcSquaredDist2d(const autoware_planning_msgs::TrajectoryPoint &a, const autoware_planning_msgs::TrajectoryPoint &b);
double calcDist2d(const geometry_msgs::Point &a, const geometry_msgs::Point &b);
double calcDist2d(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b);
double calcDist2d(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b);
double calcDist2d(const autoware_planning_msgs::TrajectoryPoint &a, const autoware_planning_msgs::TrajectoryPoint &b);
int calcClosestWaypoint(const autoware_planning_msgs::Trajectory &trajectory, const geometry_msgs::Point &point);
int calcClosestWaypoint(const autoware_planning_msgs::Trajectory &trajectory, const geometry_msgs::Pose &pose);
bool extractPathAroundIndex(const autoware_planning_msgs::Trajectory &trajectory, const int index, const double &ahead_length,
                            const double &behind_length, autoware_planning_msgs::Trajectory &extracted_base_waypoints);
bool linearInterpPath(const autoware_planning_msgs::Trajectory &base_trajectory, const int resample_num,
                      autoware_planning_msgs::Trajectory &resampled_trajectory);
double calcLengthOnWaypoints(const autoware_planning_msgs::Trajectory &trajectory, const int idx1, const int idx2);
void calcWaypointsArclength(const autoware_planning_msgs::Trajectory &trajectory, std::vector<double> &arclength);
bool scalingVelocitWithStopPoint(const autoware_planning_msgs::Trajectory &trajectory, const int &self_idx, const int &stop_idx,
                                 autoware_planning_msgs::Trajectory &trajectory_scaled);
void zeroVelocity(autoware_planning_msgs::Trajectory &trajectory);
bool calcStopDistWithConstantJerk(const double &v0, const double &a0, const double &s_lim, const double &v_end,
                                  double &t1, double &t2, double &stop_dist);
bool calcStopVelocityWithConstantJerk(const double &v0, const double &a0, const double &s, const double &t1,
                                      const double &t2, const int &start_idx, autoware_planning_msgs::Trajectory &trajectory,
                                      std::vector<double> &a_arr, std::vector<double> &s_arr);
void mininumVelocityFilter(const double &min_vel, autoware_planning_msgs::Trajectory &trajectory);
void maximumVelocityFilter(const double &max_vel, autoware_planning_msgs::Trajectory &trajectory);
void multiplyConstantToTrajectoryVelocity(const double &scalar, autoware_planning_msgs::Trajectory &trajectory);
void insertZeroVelocityAfterIdx(const int &stop_idx, autoware_planning_msgs::Trajectory &trajectory);
double getVx(const autoware_planning_msgs::Trajectory &trajectory, const int &i);
double getForwardAcc(const autoware_planning_msgs::Trajectory &trajectory, const int &i);
double getTrajectoryJerk(const autoware_planning_msgs::Trajectory &trajectory, const int idx);
 
double getDurationToNextIdx(const autoware_planning_msgs::Trajectory &trajectory, const double &v, const int &i);
bool searchZeroVelocityIdx(const autoware_planning_msgs::Trajectory &trajectory, int &idx);
bool calcTrajectoryCurvatureFrom3Points(const autoware_planning_msgs::Trajectory &trajectory, const unsigned int &idx_dist, std::vector<double> &k_arr);
bool backwardAccelerationFilterForStopPoint(const double &accel, autoware_planning_msgs::Trajectory &trajectory);
}  // namespace vpu