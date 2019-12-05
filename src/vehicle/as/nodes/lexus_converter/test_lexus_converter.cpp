#include <iostream>
#include <cstdlib>

#include "accel_map.h"
#include "brake_map.h"

int main(int argc, char** argv)
{
  std::cout << "start test!" << std::endl;

  std::cout << "create accel_map object!" << std::endl;
  AccelMap accel_map;
  accel_map.readAccelMapFromCSV("/home/makoto/.autoware/vehicle_data/lexus/accel_map_test.csv");
  std::cout << "finish reading accel_map from csv" << std::endl;
  std::cout << "--------------------------------------------------------------------------------" << std::endl;
  accel_map.showAccelMapInfo();
  std::cout << "--------------------------------------------------------------------------------" << std::endl;
  bool is_throttle;
  double throttle;
  is_throttle = accel_map.getThrottle(2.0, 10.0 * 1000 / 60 / 60, throttle);
  std::cout << "acc = 2.0 [m/s^2], vel = 10 [km/h]: expected throttle = 0.5 [-], calculated throttle = " << throttle
            << " [-] (" << is_throttle << ")" << std::endl;
  is_throttle = accel_map.getThrottle(2.0, 15.0 * 1000 / 60 / 60, throttle);
  std::cout << "acc = 2.0 [m/s^2], vel = 15 [km/h]: expected throttle = 0.75 [-], calculated throttle = " << throttle
            << " [-] (" << is_throttle << ")" << std::endl;
  is_throttle = accel_map.getThrottle(8.0, 0.0 * 1000 / 60 / 60, throttle);
  std::cout << "acc = 8.0 [m/s^2], vel = 0 [km/h]: expected throttle = 1.0 [-] (max throttle), calculated throttle = "
            << throttle << " [-] (" << is_throttle << ")" << std::endl;
  is_throttle = accel_map.getThrottle(-2.0, 10.0 * 1000 / 60 / 60, throttle);
  std::cout << "acc = -2.0 [m/s^2], vel = 10 [km/h]: expected throttle = false (need braking), calculated throttle = "
            << throttle << " [-] (" << is_throttle << ")" << std::endl;
  std::cout << "--------------------------------------------------------------------------------" << std::endl;

  std::cout << "create brake_map object!" << std::endl;
  BrakeMap brake_map;
  brake_map.readBrakeMapFromCSV("/home/makoto/.autoware/vehicle_data/lexus/brake_map_test.csv");
  std::cout << "finish reading brake_map from csv" << std::endl;
  std::cout << "--------------------------------------------------------------------------------" << std::endl;
  bool is_brake;
  double brake;
  is_brake = brake_map.getBrake(-4.0, 10.0 * 1000 / 60 / 60, brake);
  std::cout << "acc = -4.0 [m/s^2], vel = 10 [km/h]: expected brake = 0.5 [-], calculated brake = " << brake << " [-] ("
            << is_brake << ")" << std::endl;
  is_brake = brake_map.getBrake(-6.0, 15.0 * 1000 / 60 / 60, brake);
  std::cout << "acc = -6.0 [m/s^2], vel = 15 [km/h]: expected brake = 0.75 [-], calculated brake = " << brake
            << " [-] (" << is_brake << ")" << std::endl;
  is_brake = brake_map.getBrake(0, 10.0 * 1000 / 60 / 60, brake);
  std::cout << "acc = 0.0 [m/s^2], vel = 10 [km/h]: expected brake = 0.0 [-], calculated brake = " << brake << " [-] ("
            << is_brake << ")" << std::endl;
  is_brake = brake_map.getBrake(-8.0, 10.0 * 1000 / 60 / 60, brake);
  std::cout << "acc = -8.0 [m/s^2], vel = 10 [km/h]: expected brake = false, calculated brake = " << brake << " [-] ("
            << is_brake << ")" << std::endl;

  return 0;
}
