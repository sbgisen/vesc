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

#include "vesc_hw_interface/vesc_hw_interface.hpp"
#include <angles/angles.h>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>

namespace vesc_hw_interface
{
VescHwInterface::VescHwInterface()
{
  vesc_interface_ = std::make_shared<VescInterface>(
      std::string(), std::bind(&VescHwInterface::packetCallback, this, std::placeholders::_1),
      std::bind(&VescHwInterface::errorCallback, this, std::placeholders::_1));
}

CallbackReturn VescHwInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  command_ = std::numeric_limits<double>::quiet_NaN();
  position_ = std::numeric_limits<double>::quiet_NaN();
  velocity_ = std::numeric_limits<double>::quiet_NaN();
  effort_ = std::numeric_limits<double>::quiet_NaN();

  // initializes the joint name
  joint_name_ = info_.hardware_parameters["joint_name"];

  // initializes commands and states
  command_ = 0.0;
  position_ = 0.0;
  velocity_ = 0.0;
  effort_ = 0.0;

  // reads system parameters
  port_ = info_.hardware_parameters["port"];
  gear_ratio_ = std::stod(info_.hardware_parameters["gear_ratio"]);
  torque_const_ = std::stod(info_.hardware_parameters["torque_const"]);
  num_hall_sensors_ = std::stoi(info_.hardware_parameters["num_hall_sensors"]);

  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Gear ratio is set to %f", gear_ratio_);
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Torque constant is set to %f", torque_const_);

  // reads driving mode setting
  // - assigns an empty string if param. is not found

  num_rotor_poles_ = std::stoi(info_.hardware_parameters["num_rotor_poles"]);

  if (num_rotor_poles_ % 2 != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "There should be even number of rotor poles");
    rclcpp::shutdown();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "The number of motor pole pairs is set to %d", num_rotor_poles_);

  command_mode_ = info_.hardware_parameters["command_mode"];
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "mode: %s", command_mode_.data());

  // check joint type
  joint_type_ = info_.hardware_parameters["joint_type"];
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "joint type: %s", joint_type_.data());
  if ((joint_type_ != "revolute") && (joint_type_ != "continuous") && (joint_type_ != "prismatic"))
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"), "Verify your joint type");
    return CallbackReturn::ERROR;
  }

  const hardware_interface::ComponentInfo& joint = info_.joints[0];
  joint_name_ = joint.name;
  if (joint.command_interfaces.size() != 3)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"), "Joint '%s' has %zu command interfaces found. 3 expected.",
                 joint.name.c_str(), joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }
  std::vector<std::string> command_interface_order = { hardware_interface::HW_IF_POSITION,
                                                       hardware_interface::HW_IF_VELOCITY,
                                                       hardware_interface::HW_IF_EFFORT };
  for (size_t i = 0; i < joint.command_interfaces.size(); ++i)
  {
    if (joint.command_interfaces[i].name != command_interface_order[i])
    {
      RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"),
                   "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.command_interfaces[i].name.c_str(), command_interface_order[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (joint.state_interfaces.size() != 3)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                 joint.name.c_str(), joint.state_interfaces.size());
    return CallbackReturn::ERROR;
  }
  std::vector<std::string> state_interface_order = { hardware_interface::HW_IF_POSITION,
                                                     hardware_interface::HW_IF_VELOCITY,
                                                     hardware_interface::HW_IF_EFFORT };
  for (size_t i = 0; i < joint.state_interfaces.size(); ++i)
  {
    if (joint.state_interfaces[i].name != state_interface_order[i])
    {
      RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"),
                   "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[i].name.c_str(), state_interface_order[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Successfully Initialized!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn VescHwInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "connect to %s", port_.c_str());
    vesc_interface_->connect(port_);
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "connected");
  }
  catch (const vesc_driver::SerialException& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"), "Failed to connect to the VESC, %s.", exception.what());
    return CallbackReturn::FAILURE;
  }

  if (command_mode_ == hardware_interface::HW_IF_POSITION)
  {
    // initializes the servo controller
    servo_controller_.init(info_, vesc_interface_);
    servo_controller_.setGearRatio(gear_ratio_);
    servo_controller_.setTorqueConst(torque_const_);
    servo_controller_.setRotorPoles(num_rotor_poles_);
    servo_controller_.setHallSensors(num_hall_sensors_);
    servo_controller_.setJointType(joint_type_ == "revolute" ? 0 : joint_type_ == "continuous" ? 1 : 2);
    screw_lead_ = std::stod(info_.hardware_parameters["screw_lead"]);
    servo_controller_.setScrewLead(screw_lead_);
  }

  if (command_mode_ == "velocity_duty")
  {
    // initializes the wheel controller
    wheel_controller_.init(info_, vesc_interface_);
    wheel_controller_.setGearRatio(gear_ratio_);
    wheel_controller_.setTorqueConst(torque_const_);
    wheel_controller_.setRotorPoles(num_rotor_poles_);
    wheel_controller_.setHallSensors(num_hall_sensors_);
  }

  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn VescHwInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescHwInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescHwInterface::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VescHwInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity_));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &effort_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescHwInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &command_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &command_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &command_));

  return command_interfaces;
}

CallbackReturn VescHwInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Set some default values
  if (std::isnan(position_))
    position_ = 0;
  if (std::isnan(velocity_))
    velocity_ = 0;
  if (std::isnan(effort_))
    effort_ = 0;
  if (std::isnan(command_))
    command_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "System successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescHwInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type VescHwInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // requests joint states
  // function `packetCallback` will be called after receiving return packets
  if (command_mode_ == "position")
  {
    // For PID control, request packets are automatically sent in the control cycle.
    // The latest data is read in this function.
    vesc_interface_->requestState();
    position_ = servo_controller_.getPositionSens();
    velocity_ = servo_controller_.getVelocitySens();
    effort_ = servo_controller_.getEffortSens();
  }
  else if (command_mode_ == "velocity_duty")
  {
    vesc_interface_->requestState();
    position_ = wheel_controller_.getPositionSens();
    velocity_ = wheel_controller_.getVelocitySens();
    effort_ = wheel_controller_.getEffortSens();
  }
  else
  {
    vesc_interface_->requestState();
  }

  if (joint_type_ == "revolute")
  {
    position_ = angles::normalize_angle(position_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescHwInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  // sends commands
  if (command_mode_ == "position")
  {
    // Limit the speed using the parameters listed in xacro
    // limit_position_interface_.enforceLimits(period);
    // limit_position_handle_.enforceLimits(period);

    // executes PID control
    servo_controller_.setTargetPosition(command_);
    servo_controller_.control(1.0 / period.seconds());
  }
  else if (command_mode_ == "velocity")
  {
    // limit_velocity_interface_.enforceLimits(period);

    // converts the velocity unit: rad/s or m/s -> rpm -> erpm
    const double command_rpm = command_ * 60.0 / 2.0 / M_PI / gear_ratio_;
    const double command_erpm = command_rpm * static_cast<double>(num_rotor_poles_) / 2;

    // sends a reference velocity command
    vesc_interface_->setSpeed(command_erpm);
  }
  else if (command_mode_ == "velocity_duty")
  {
    // limit_velocity_interface_.enforceLimits(period);

    // executes PID control
    wheel_controller_.setTargetVelocity(command_);
    wheel_controller_.control(1.0 / period.seconds());
  }
  else if (command_mode_ == "effort")
  {
    // limit_effort_interface_.enforceLimits(period);

    // converts the command unit: Nm or N -> A
    const double command_current = command_ * gear_ratio_ / torque_const_;

    // sends a reference current command
    vesc_interface_->setCurrent(command_current);
  }
  else if (command_mode_ == "effort_duty")
  {
    command_ = std::max(-1.0, command_);
    command_ = std::min(1.0, command_);

    // sends a  duty command
    vesc_interface_->setDutyCycle(command_);
  }
  return hardware_interface::return_type::OK;
}

rclcpp::Time VescHwInterface::getTime() const
{
  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  return clock.now();
}

void VescHwInterface::packetCallback(const std::shared_ptr<VescPacket const>& packet)
{
  if (!vesc_interface_->isRxDataUpdated())
  {
    RCLCPP_WARN(rclcpp::get_logger("VescHwInterface"), "[VescHwInterface::packetCallback]packetCallcack called, but "
                                                       "no packet received");
  }
  if (command_mode_ == "position")
  {
    servo_controller_.updateSensor(packet);
  }
  else if (command_mode_ == "velocity_duty")
  {
    wheel_controller_.updateSensor(packet);
  }
  else if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);

    const double current = values->getMotorCurrent();
    const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_rotor_poles_ / 2);
    const double steps = values->getPosition();

    position_ = steps / (num_hall_sensors_ * num_rotor_poles_) * gear_ratio_;  // unit: rad or m
    velocity_ = velocity_rpm / 60.0 * 2.0 * M_PI * gear_ratio_;                // unit: rad/s or m/s
    effort_ = current * torque_const_ / gear_ratio_;                           // unit: Nm or N
  }

  return;
}

void VescHwInterface::errorCallback(const std::string& error)
{
  RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "%s", error.c_str());
  return;
}

}  // namespace vesc_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::VescHwInterface, hardware_interface::ActuatorInterface)
