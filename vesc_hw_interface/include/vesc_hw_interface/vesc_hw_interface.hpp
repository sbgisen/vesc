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

#ifndef VESC_HW_INTERFACE_VESC_HW_INTERFACE_HPP_
#define VESC_HW_INTERFACE_VESC_HW_INTERFACE_HPP_

#include "visibility_control.h"
#include <hardware_interface/actuator_interface.hpp>
// #include <joint_limits_interface/joint_limits.h>
// #include <joint_limits_interface/joint_limits_interface.h>
// #include <joint_limits_interface/joint_limits_rosparam.h>
// #include <joint_limits_interface/joint_limits_urdf.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "vesc_driver/vesc_interface.hpp"
#include "vesc_hw_interface/vesc_servo_controller.hpp"
#include "vesc_hw_interface/vesc_wheel_controller.hpp"

namespace vesc_hw_interface
{
using vesc_driver::VescInterface;
using vesc_driver::VescPacket;
using vesc_driver::VescPacketValues;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class VescHwInterface : public hardware_interface::ActuatorInterface
{
public:
  VescHwInterface();

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  rclcpp::Time getTime() const;

private:
  std::shared_ptr<VescInterface> vesc_interface_;
  VescServoController servo_controller_;
  VescWheelController wheel_controller_;

  std::string joint_name_, command_mode_, port_;
  std::string joint_type_;

  double command_;
  double position_, velocity_, effort_;  // joint states

  int num_rotor_poles_;               // the number of rotor poles
  int num_hall_sensors_;              // the number of hall sensors
  double gear_ratio_, torque_const_;  // physical params
  double screw_lead_;                 // linear distance (m) of 1 revolution

  // joint_limits_interface::PositionJointSaturationInterface limit_position_interface_;
  // joint_limits_interface::VelocityJointSaturationInterface limit_velocity_interface_;
  // joint_limits_interface::EffortJointSaturationInterface limit_effort_interface_;

  void packetCallback(const std::shared_ptr<VescPacket const>&);
  void errorCallback(const std::string&);
};

}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_HW_INTERFACE_HPP_
