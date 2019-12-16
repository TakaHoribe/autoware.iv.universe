/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "accel_map.h"

AccelMap::AccelMap()
{
}

AccelMap::~AccelMap()
{
}

bool AccelMap::readAccelMapFromCSV(std::string csv_path)
{
  Csv csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCsv(table))
  {
    ROS_ERROR("[Accel Map] Cannot open %s", csv_path.c_str());
    return false;
  }

  if (table[0].size() < 2)
  {
    ROS_ERROR("[Accel Map] Cannot read %s. CSV file should have at least 2 column", csv_path.c_str());
    return false;
  }
  vehicle_name_ = table[0][0];
  for (unsigned int i = 1; i < table[0].size(); i++)
  {
    vel_index_.push_back(std::stod(table[0][i]) * 1000 / 60 / 60);  // km/h => m/s
  }

  for (unsigned int i = 1; i < table.size(); i++)
  {
    if (table[0].size() != table[i].size())
    {
      ROS_ERROR("[Accel Map] Cannot read %s. Each row should have a same number of columns", csv_path.c_str());
      return false;
    }
    throttle_index_.push_back(std::stod(table[i][0]));
    std::vector<double> accs;
    for (unsigned int j = 1; j < table[i].size(); j++)
    {
      accs.push_back(std::stod(table[i][j]));
    }
    accel_map_.push_back(accs);
  }

  return true;
}

bool AccelMap::getThrottle(double acc, double vel, double& throttle)
{
  LinearInterpolate linear_interp;
  std::vector<double> accs_interpolated;

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accs : accel_map_)
  {
    double acc_interpolated;
    linear_interp.interpolate(vel_index_, accs, vel, acc_interpolated);
    accs_interpolated.push_back(acc_interpolated);
  }

  // calculate throttle
  // When the desired acceleration is smaller than the throttle area, return false => brake sequence
  // When the desired acceleration is greater than the throttle area, return max throttle
  if (acc < accs_interpolated.front())
  {
    return false;
  }
  else if (accs_interpolated.back() < acc)
  {
    throttle = throttle_index_.back();
    return true;
  }
  linear_interp.interpolate(accs_interpolated, throttle_index_, acc, throttle);

  return true;
}

void AccelMap::showAccelMapInfo()
{
  std::cout << "Accel Map Information" << std::endl;

  std::cout << "Vehicle name: " << vehicle_name_ << std::endl;

  std::cout << "Velocity indexes: ";
  for (double v : vel_index_)
  {
    std::cout << v << " ";
  }
  std::cout << std::endl;

  std::cout << "Throttle indexes: ";
  for (double t : throttle_index_)
  {
    std::cout << t << " ";
  }
  std::cout << std::endl;

  std::cout << "Acceleration Map:" << std::endl;
  for (std::vector<double> accs : accel_map_)
  {
    for (double acc : accs)
    {
      std::cout << acc << " ";
    }
    std::cout << std::endl;
  }
}
