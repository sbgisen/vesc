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

#ifndef VESC_HW_INTERFACE_VESC_WHEEL_CONTROLLER_HPP_
#define VESC_HW_INTERFACE_VESC_WHEEL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <vesc_driver/vesc_interface.hpp>
#include "vesc_hw_interface/vesc_step_difference.hpp"

namespace vesc_hw_interface
{
using vesc_driver::VescInterface;
using vesc_driver::VescPacket;
using vesc_driver::VescPacketValues;
using vesc_step_difference::VescStepDifference;

class VescWheelController
{
public:
  void init(hardware_interface::HardwareInfo& info, const std::shared_ptr<VescInterface>& interface);
  void control(const double control_rate);
  void setTargetVelocity(const double velocity);
  void setGearRatio(const double gear_ratio);
  void setTorqueConst(const double torque_const);
  void setRotorPoles(const int rotor_poles);
  void setHallSensors(const int hall_sensors);
  double getPositionSens();
  double getVelocitySens();
  double getEffortSens();
  void updateSensor(const std::shared_ptr<const VescPacket>& packet);

private:
  std::shared_ptr<VescInterface> interface_ptr_;
  VescStepDifference vesc_step_difference_;

  double kp_, ki_, kd_;
  double i_clamp_;
  bool antiwindup_;
  double duty_limiter_;
  double num_rotor_pole_pairs_, num_rotor_poles_;
  double num_hall_sensors_;
  double gear_ratio_, torque_const_;

  double target_velocity_;
  double position_steps_;
  int prev_steps_;
  double position_sens_;
  double velocity_sens_;
  double effort_sens_;

  double error_, error_dt_, error_integ_;
  double target_steps_;
  bool pid_initialize_;
  bool sensor_initialize_;

  double control_rate_;
  // rclcpp::Timer control_timer_;
  // void controlTimerCallback(const ros::TimerEvent& e);
};
}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_WHEEL_CONTROLLER_HPP_
