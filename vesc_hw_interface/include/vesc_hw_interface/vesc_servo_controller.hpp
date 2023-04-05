/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#ifndef VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_HPP_
#define VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_HPP_

#include <rclcpp/clock.hpp>
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

class VescServoController
{
public:
  VescServoController();
  ~VescServoController();

  void init(hardware_interface::HardwareInfo& info, const std::shared_ptr<VescInterface>& interface);
  void control(const double control_rate);
  void setTargetPosition(const double position);
  void setGearRatio(const double gear_ratio);
  void setTorqueConst(const double torque_const);
  void setRotorPoles(const int rotor_poles);
  void setHallSensors(const int hall_sensors);
  void setJointType(const int joint_type);
  void setScrewLead(const double screw_lead);
  double getZeroPosition() const;
  double getPositionSens();
  double getVelocitySens();
  double getEffortSens();
  void executeCalibration();
  void updateSensor(const std::shared_ptr<VescPacket const>& packet);

private:
  std::shared_ptr<VescInterface> interface_ptr_;
  VescStepDifference vesc_step_difference_;

  const std::string DUTY_ = "duty";
  const std::string CURRENT_ = "current";

  bool calibration_flag_;
  double calibration_current_;    // unit: A
  double calibration_duty_;       // 0.0 ~ 1.0
  std::string calibration_mode_;  // "duty" or "current" (default: "current")
  double calibration_position_;   // unit: rad or m
  double zero_position_;          // unit: rad or m
  double kp_, ki_, kd_;
  double i_clamp_, duty_limiter_;
  bool antiwindup_;
  double control_rate_;
  int num_rotor_poles_;               // the number of rotor poles
  int num_hall_sensors_;              // the number of hall sensors
  double gear_ratio_, torque_const_;  // physical params
  double screw_lead_;                 // linear distance (m) of 1 revolution
  int joint_type_;
  // ros::Timer control_timer_;
  // Internal variables for PID control
  double target_position_;
  double target_position_previous_;
  double sens_position_, sens_velocity_, sens_effort_;
  double position_steps_;
  int32_t steps_previous_;
  double error_integ_;
  // Internal variables for initialization
  bool sensor_initialize_;
  int calibration_steps_;
  double calibration_previous_position_;

  bool calibrate();
  // void controlTimerCallback(const ros::TimerEvent& e);
};

}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_HPP_
