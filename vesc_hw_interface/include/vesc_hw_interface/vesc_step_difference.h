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

#ifndef VESC_STEP_DIFFERENCE_H_
#define VESC_STEP_DIFFERENCE_H_

#include <ros/ros.h>
namespace vesc_step_difference
{
class VescStepDifference
{
public:
  VescStepDifference();
  ~VescStepDifference();
  double getStepDifference(const double step_in, bool reset);
  void enableSmooth(double control_rate, double max_sampling_time, int max_step_diff);

private:
  double stepDifferenceRaw(const double step_in, bool reset);
  double stepDifferenceVariableWindow(const double step_in, bool reset);

  // Enable smooth difference
  bool enable_smooth_;
  // Params for counterTDRaw
  int16_t step_diff_raw_prev_;
  // Params for counterTDVariableWindow
  int step_diff_vw_max_window_size_;
  int step_diff_vw_max_step_;
  std::vector<int16_t> step_diff_log_;
};
}  // namespace vesc_step_difference

#endif