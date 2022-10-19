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

#ifndef VESC_HW_INTERFACE_VESC_WHEEL_CONTROLLER_H_
#define VESC_HW_INTERFACE_VESC_WHEEL_CONTROLLER_H_

#include <ros/ros.h>
#include <vesc_driver/vesc_interface.h>

namespace vesc_hw_interface
{
using vesc_driver::VescInterface;

class VescWheelController
{
public:
  void init(ros::NodeHandle nh, VescInterface* vesc_interface);
  void control(const double target_velocity, const double current_pulse, bool initialize);
  void setControlFrequency(const double frequency);

private:
  VescInterface* interface_ptr_;

  double ctrl_frequency_;
  double kp_, ki_, kd_;
  double i_clamp_;
  bool antiwindup_;
  double duty_limiter_;
  int num_motor_pole_pairs_;

  double error_, error_dt_, error_integ_, error_integ_prev_;
  double target_pulse_;

  double counterTD(const double count_in, bool initialize);
  uint16_t counter_changed_log_[10][2];
  double counter_td_tmp_[10];
  uint16_t counter_changed_single_;
};
}  // namespace vesc_hw_interface

#endif
