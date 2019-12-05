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

#include "pid.h"

PIDController::PIDController()
{
  error_integral_ = 0;
  prev_error_ = 0;
  first_time_ = true;
}

double PIDController::calculate(double error, double dt)
{
  error_integral_ += error * dt;
  
  double error_differential;
  if (first_time_)
  {
    error_differential = 0;
    first_time_ = false;
  }
  else
  {
    error_differential = (error - prev_error_) / dt;
  }

  prev_error_ = error;

  return kp_ * error + ki_ * error_integral_ + kd_ * error_differential;
}

void PIDController::init(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::reset()
{
  error_integral_ = 0;
  prev_error_ = 0;
  first_time_ = true;
}
