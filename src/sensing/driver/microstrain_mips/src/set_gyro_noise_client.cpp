#include <cstdlib>
#include "microstrain_mips/SetGyroNoise.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_gyro_noise_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<microstrain_mips::SetGyroNoise>("SetGyroNoise");
  microstrain_mips::SetGyroNoise srv;

  srv.request.noise.x = atoll(argv[1]);
  srv.request.noise.y = atoll(argv[2]);
  srv.request.noise.z = atoll(argv[3]);

  if (client.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("success");
    }
  } else {
    ROS_INFO("Failed to call service");
  }
  return 0;
}
