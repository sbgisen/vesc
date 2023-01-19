/*********************************************************************
 * Copyright (c) 2022 SoftBank Corp.
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
 *
 ********************************************************************/

#include "vesc_hw_interface/vesc_step_difference.h"
namespace vesc_step_difference
{
VescStepDifference::VescStepDifference()
{
}

VescStepDifference::~VescStepDifference()
{
}

void VescStepDifference::enableSmooth(double control_rate, double max_sampling_time, int max_step_diff)
{
  enable_smooth_ = true;
  step_diff_vw_max_step_ = max_step_diff;
  step_diff_vw_max_window_size_ = std::max(1, static_cast<int>(control_rate * max_sampling_time));
  step_diff_log_.resize(step_diff_vw_max_window_size_ + 1);
  return;
}

double VescStepDifference::getStepDifference(const double step_in, bool reset)
{
  double output;
  if (enable_smooth_)
  {
    output = this->stepDifferenceVariableWindow(step_in, reset);
  }
  else
  {
    output = this->stepDifferenceRaw(step_in, reset);
  }
  return output;
}

double VescStepDifference::stepDifferenceRaw(const double step_in, bool reset)
{
  if (reset)
  {
    step_diff_raw_prev_ = static_cast<int16_t>(step_in);
    return 0.0;
  }
  int16_t step_diff = static_cast<int16_t>(step_in) - step_diff_raw_prev_;
  double output = static_cast<double>(step_diff);
  step_diff_raw_prev_ = static_cast<int16_t>(step_in);
  return output;
}

double VescStepDifference::stepDifferenceVariableWindow(const double step_in, bool reset)
{
  if (reset)
  {
    for (int i = 0; i < step_diff_log_.size(); i++)
    {
      step_diff_log_[i] = static_cast<int16_t>(step_in);
    }
    return 0.0;
  }
  // Increment buffer
  for (int i = step_diff_log_.size() - 1; i > 0; i--)
  {
    step_diff_log_[i] = step_diff_log_[i - 1];
  }
  step_diff_log_[0] = static_cast<int16_t>(step_in);
  // Calculate window size
  int latest_step_diff = abs(step_diff_log_[0] - step_diff_log_[1]);
  int window_size = step_diff_vw_max_window_size_;
  if (latest_step_diff > step_diff_vw_max_step_ - 1)
  {
    window_size = 1;
  }
  else if (latest_step_diff > 0)
  {
    window_size = step_diff_vw_max_window_size_ *
                  (1.0 - static_cast<double>(latest_step_diff) / static_cast<double>(step_diff_vw_max_step_));
  }
  // Get output
  int16_t step_diff = step_diff_log_[0] - step_diff_log_[window_size];
  double output = static_cast<double>(step_diff) / static_cast<double>(window_size);
  // ROS_WARN("[VescStepDifference]window,step0,step1,latest_step_diff,output: %d, %d, %d, %d, %f", window_size,
  //          step_diff_log_[0], step_diff_log_[window_size], latest_step_diff, output);
  return output;
}

}  // namespace vesc_step_difference
