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

#ifndef BRAKE_MAP_H
#define BRAKE_MAP_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include "csv.h"
#include "interpolate.h"

class BrakeMap
{
public:
  BrakeMap();
  ~BrakeMap();

  bool readBrakeMapFromCSV(std::string csv_path);
  bool getBrake(double acc, double vel, double& brake);
  void showBrakeMapInfo();

private:
  std::string vehicle_name_;
  std::vector<double> vel_index_;
  std::vector<double> brake_index_;
  std::vector<double> brake_index_rev_;
  std::vector<std::vector<double>> brake_map_;
};

#endif
