#include <cstdlib>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "get_sensor_vehicle_frame_offset");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("GetSensorVehicleFrameOffset");
  std_srvs::Trigger srv;

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("success");
    }
  } else {
    ROS_INFO("Failed to call service");
  }
  return 0;
}
