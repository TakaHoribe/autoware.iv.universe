/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef VELOCITY_CONTROLLER_PID
#define VELOCITY_CONTROLLER_PID

class PIDController
{
public:
  PIDController();
  ~PIDController() = default;

  double calculate(double error, double dt);
  void init(double kp, double ki, double kd);
  void reset();

private:
  double kp_, ki_, kd_;
  double error_integral_, prev_error_;
  bool first_time_;
};

#endif
