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
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/utilities.hpp>
#include <tinyxml2.h>

namespace vesc_hw_interface
{
VescHwInterface::VescHwInterface()
{
  vesc_interface_ = std::make_shared<VescInterface>(
      std::string(), int(), int(), std::bind(&VescHwInterface::packetCallback, this, std::placeholders::_1),
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

  // initializes commands and states
  position_ = 0.0;
  velocity_ = 0.0;
  effort_ = 0.0;
  sensor_initialize_ = false;
  position_steps_ = 0.0;

  // reads system parameters
  port_ = info_.hardware_parameters["port"];
  gear_ratio_ = 1.0;
  if (info_.hardware_parameters.find("gear_ratio") != info_.hardware_parameters.end())
  {
    gear_ratio_ = std::stod(info_.hardware_parameters["gear_ratio"]);
  }
  torque_const_ = 1.0;
  if (info_.hardware_parameters.find("torque_const") != info_.hardware_parameters.end())
  {
    torque_const_ = std::stod(info_.hardware_parameters["torque_const"]);
  }
  num_hall_sensors_ = 3;
  if (info_.hardware_parameters.find("num_hall_sensors") != info_.hardware_parameters.end())
  {
    num_hall_sensors_ = std::stoi(info_.hardware_parameters["num_hall_sensors"]);
  }
  screw_lead_ = 1.0;
  if (info_.hardware_parameters.find("screw_lead") != info_.hardware_parameters.end())
  {
    screw_lead_ = std::stod(info_.hardware_parameters["screw_lead"]);
  }
  if (info_.hardware_parameters.find("controller_id") != info_.hardware_parameters.end())
  {
    controller_id_ = std::stoi(info_.hardware_parameters["controller_id"]);
  }
  if (info_.hardware_parameters.find("vesc_id") != info_.hardware_parameters.end())
  {
    vesct_id_ = std::stoi(info_.hardware_parameters["vesc_id"]);
  }

  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Gear ratio is set to %f", gear_ratio_);
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Torque constant is set to %f", torque_const_);

  // reads driving mode setting
  // - assigns an empty string if param. is not found

  num_rotor_poles_ = 2;
  if (info_.hardware_parameters.find("num_rotor_poles") != info_.hardware_parameters.end())
  {
    num_rotor_poles_ = std::stoi(info_.hardware_parameters["num_rotor_poles"]);
  }

  if (num_rotor_poles_ % 2 != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "There should be even number of rotor poles");
    rclcpp::shutdown();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "The number of motor pole pairs is set to %d", num_rotor_poles_);

  command_mode_ = info_.hardware_parameters["command_mode"];
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "mode: %s", command_mode_.data());

  const hardware_interface::ComponentInfo& joint = info_.joints[0];
  joint_name_ = joint.name;

  // parse URDF for joint type
  auto urdf = info_.original_xml;
  if (!urdf.empty())
  {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to parse URDF XML");
      return hardware_interface::CallbackReturn::ERROR;
    }
    const tinyxml2::XMLElement* joint_it = doc.RootElement()->FirstChildElement("joint");
    while (joint_it)
    {
      const tinyxml2::XMLAttribute* name_attr = joint_it->FindAttribute("name");
      const tinyxml2::XMLAttribute* type_attr = joint_it->FindAttribute("type");
      if (name_attr && type_attr)
      {
        std::string name = joint_it->Attribute("name");
        std::string type = joint_it->Attribute("type");
        if (name == joint_name_)
        {
          joint_type_ = type;
          break;
        }
      }
      joint_it = joint_it->NextSiblingElement("joint");
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "joint type: %s", joint_type_.data());
  if ((joint_type_ != "revolute") && (joint_type_ != "continuous") && (joint_type_ != "prismatic"))
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"), "Verify your joint type");
    return CallbackReturn::ERROR;
  }

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
    vesc_interface_->connect(port_, controller_id_, vesct_id_);
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "connected");
  }
  catch (const vesc_driver::SerialException& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescHwInterface"), "Failed to connect to the VESC, %s.", exception.what());
    return CallbackReturn::FAILURE;
  }

  if ((command_mode_ == hardware_interface::HW_IF_POSITION) || (command_mode_ == hardware_interface::HW_IF_VELOCITY) ||
      (command_mode_ == hardware_interface::HW_IF_EFFORT))
  {
    vesc_interface_->requestMCConfiguration();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  upper_limit_ = 0.0;
  lower_limit_ = 0.0;
  homing_offset_ = 0.0;
  homing_done_ = false;
  homing_enabled_ = false;
  if (command_mode_ == hardware_interface::HW_IF_POSITION || command_mode_ == "position_duty")
  {
    // parse URDF for limit parameters
    auto joint_limit_itr = info_.limits.find(joint_name_);
    if (joint_limit_itr != info_.limits.end())
    {
      upper_limit_ = joint_limit_itr->second.max_position;
      lower_limit_ = joint_limit_itr->second.min_position;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("VescHwInterface"), "No joint position limits found in URDF, using default "
                                                         "limits");
    }

    // initializes the servo controller
    servo_controller_.init(info_, vesc_interface_, gear_ratio_, torque_const_, num_rotor_poles_, num_hall_sensors_,
                           joint_type_ == "revolute"   ? 0 :
                           joint_type_ == "continuous" ? 1 :
                                                         2,
                           screw_lead_, upper_limit_, lower_limit_);

    auto calibration_params = servo_controller_.getCalibrationParameters();
    homing_enabled_ = calibration_params.enable_calibration;
    homing_position_ = calibration_params.calibration_position;
    if (homing_enabled_)
    {
      while (rclcpp::ok())
      {
        vesc_interface_->requestState();
        servo_controller_.spinSensorData();
        if (servo_controller_.calibrate())
          break;
        rclcpp::sleep_for(std::chrono::milliseconds(10));
      }
    }
    homing_done_ = true;
    if (command_mode_ == "position_duty")
    {
      position_ = servo_controller_.getPositionSens();
      velocity_ = servo_controller_.getVelocitySens();
      effort_ = servo_controller_.getEffortSens();
    }
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
  if (command_mode_ == "position_duty")
  {
    // For PID control, request packets are automatically sent in the control cycle.
    // The latest data is read in this function.
    vesc_interface_->requestState();
    servo_controller_.spinSensorData();
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

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescHwInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  // sends commands

  auto command = command_;
  if (std::isnan(command) && command_mode_ != "position_duty")
  {
    command = 0.0;
  }
  if (command_mode_ == "position_duty")
  {
    // Limit the speed using the parameters listed in xacro
    // limit_position_interface_.enforceLimits(period);
    // limit_position_handle_.enforceLimits(period);

    // executes PID control
    servo_controller_.setTargetPosition(command);
    servo_controller_.control(1.0 / period.seconds());
  }
  else if (command_mode_ == "position")
  {
    command = VESC_POS_MAPPING_RANGE * (command - homing_position_) / (upper_limit_ - lower_limit_);
    command = std::fmod(command + homing_offset_ + VESC_POS_RANGE, VESC_POS_RANGE);
    vesc_interface_->setPosition(command);
  }
  else if (command_mode_ == "velocity")
  {
    // limit_velocity_interface_.enforceLimits(period);

    // converts the velocity unit: rad/s or m/s -> rpm -> erpm
    const double command_rpm = command * 60.0 / 2.0 / M_PI / gear_ratio_;
    const double command_erpm = command_rpm * static_cast<double>(num_rotor_poles_) / 2;

    // sends a reference velocity command
    vesc_interface_->setSpeed(command_erpm);
  }
  else if (command_mode_ == "velocity_duty")
  {
    // limit_velocity_interface_.enforceLimits(period);

    // executes PID control
    wheel_controller_.setTargetVelocity(command);
    wheel_controller_.control(1.0 / period.seconds());
  }
  else if (command_mode_ == "effort")
  {
    // limit_effort_interface_.enforceLimits(period);

    // converts the command unit: Nm or N -> A
    const double command_current = command * gear_ratio_ / torque_const_;

    // sends a reference current command
    vesc_interface_->setCurrent(command_current);
  }
  else if (command_mode_ == "effort_duty")
  {
    command = std::max(-1.0, command);
    command = std::min(1.0, command);

    // sends a  duty command
    vesc_interface_->setDutyCycle(command);
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
  if (command_mode_ == "position_duty")
  {
    servo_controller_.updateSensor(packet);
    return;
  }
  if (command_mode_ == "velocity_duty")
  {
    wheel_controller_.updateSensor(packet);
    return;
  }

  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);

    const auto current = values->getMotorCurrent();
    const auto velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_rotor_poles_ / 2);
    const auto position = values->getPosition();
    const auto steps = static_cast<int32_t>(values->getTachometer());

    if (!homing_done_ && homing_enabled_)
    {
      servo_controller_.updateSensor(packet);
      homing_offset_ = position;
      return;
    }
    if (!sensor_initialize_)
    {
      if (joint_type_ == "revolute" || joint_type_ == "prismatic")
      {
        if (!homing_enabled_)
        {
          sensor_initialize_ = true;
          return;
        }
        sensor_initialize_ =
            (std::fabs(homing_offset_ - position) < std::numeric_limits<double>::epsilon()) ? true : false;
        homing_offset_ = position;
      }
      else if (joint_type_ == "continuous")
      {
        sensor_initialize_ = (steps == prev_steps_) ? true : false;
        prev_steps_ = steps;
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("VescHwInterface"), "waiting for values to settle...");
      return;
    }

    // calculate position
    if (joint_type_ == "revolute" || joint_type_ == "prismatic")
    {
      // `position` is [deg] but here we mapped the position to the joint limits hence unit is irrelevant
      position_ = std::fmod(position - homing_offset_ + VESC_POS_RANGE, VESC_POS_RANGE);
      if (position_ > VESC_POS_WRAP_THRESHOLD)
      {
        position_ -= VESC_POS_RANGE;
      }
      position_ = homing_position_ + position_ * (upper_limit_ - lower_limit_) / VESC_POS_MAPPING_RANGE;
    }
    else if (joint_type_ == "continuous")
    {
      // use tachometer to calculate position
      position_steps_ += static_cast<double>(steps - prev_steps_);
      prev_steps_ = steps;
      position_ = (position_steps_ * 2.0 * M_PI) / (num_rotor_poles_ * 3.0) * gear_ratio_;  // unit: rad
    }

    // calculate velocity
    if (joint_type_ == "revolute" || joint_type_ == "continuous")
    {
      velocity_ = (velocity_rpm * gear_ratio_) / 60.0 * 2.0 * M_PI;  // unit: rad/s
    }
    else if (joint_type_ == "prismatic")
    {
      velocity_ = (velocity_rpm * gear_ratio_) / 60.0;  // unit: m/s
    }

    // calculate effort
    effort_ = current * torque_const_ / gear_ratio_;  // unit: Nm or N
  }
}

void VescHwInterface::errorCallback(const std::string& error)
{
  RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "%s", error.c_str());
  return;
}

}  // namespace vesc_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::VescHwInterface, hardware_interface::ActuatorInterface)
