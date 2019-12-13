#include <behavior_velocity_planner/api.hpp>

namespace behavior_planning
{
bool getDynemicObjects(std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> &objects)
{
    return SingletonDataManager::getInstance().getDynemicObjects(objects);
}

bool getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const> &pointcloud)
{
    return SingletonDataManager::getInstance().getNoGroundPointcloud(pointcloud);
}

bool getCurrentSelfVelocity(std::shared_ptr<geometry_msgs::TwistStamped const> &twist){
    return SingletonDataManager::getInstance().getCurrentSelfVelocity(twist);
}

bool getBaselink2FrontLength(double &length){
    double front_overhang;
    double wheel_base;
    if (!SingletonDataManager::getInstance().getWheelBase(wheel_base) ||
        !SingletonDataManager::getInstance().getFrontOverhang(front_overhang))
        return false;
    length = front_overhang + wheel_base;
    return true;
}

bool isVehicleStopping(){
    std::shared_ptr<geometry_msgs::TwistStamped const> twist_ptr;
    getCurrentSelfVelocity(twist_ptr);
    if (twist_ptr == nullptr)
    return false;
    return twist_ptr->twist.linear.x < 0.1 ? true : false;
}

bool getCurrentSelfPose(geometry_msgs::PoseStamped &pose)
{
    return SingletonDataManager::getInstance().getCurrentSelfPose(pose);
}

bool getLaneletMap(lanelet::LaneletMapConstPtr &lanelet_map_pt,
                   lanelet::routing::RoutingGraphConstPtr &routing_graph_ptr)
{
    return SingletonDataManager::getInstance().getLaneletMap(lanelet_map_pt, routing_graph_ptr);
}
}