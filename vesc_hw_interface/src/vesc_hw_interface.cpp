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

#include "vesc_hw_interface/vesc_hw_interface.h"
#include <urdf_model/joint.h>
#include <cmath>
#include "ros/forwards.h"

namespace vesc_hw_interface
{
VescHwInterface::VescHwInterface()
  : vesc_interface_(std::string(), std::bind(&VescHwInterface::packetCallback, this, std::placeholders::_1),
                    std::bind(&VescHwInterface::errorCallback, this, std::placeholders::_1))
{
}

VescHwInterface::~VescHwInterface()
{
}

bool VescHwInterface::init(ros::NodeHandle& nh_root, ros::NodeHandle& nh)
{
  // reads a port name to open
  std::string port;
  if (!nh.getParam("port", port))
  {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return false;
  }

  // attempts to open the serial port
  try
  {
    vesc_interface_.connect(port);
  }
  catch (serial::SerialException exception)
  {
    ROS_FATAL("Failed to connect to the VESC, %s.", exception.what());
    ros::shutdown();
    return false;
  }

  // initializes the joint name
  nh.param<std::string>("joint_name", joint_name_, "joint_vesc");

  // loads joint limits
  std::string robot_description_name, robot_description;
  nh.param<std::string>("robot_description_name", robot_description_name, "/robot_description");

  // parses the urdf
  std::string joint_type;
  command_mode_ = "";
  if (nh.getParam(robot_description_name, robot_description))
  {
    const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(robot_description);
    const urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(joint_name_);

    if (getJointLimits(urdf_joint, joint_limits_))
    {
      ROS_INFO("Joint limits are loaded");
    }

    // extract joint type
    switch (urdf_joint->type)
    {
      case urdf::Joint::REVOLUTE:
        joint_type = "revolute";
        break;
      case urdf::Joint::CONTINUOUS:
        joint_type = "continuous";
        break;
      case urdf::Joint::PRISMATIC:
        joint_type = "prismatic";
        break;
    }

    // extract hardware interface type
    transmission_interface::TransmissionParser parser;
    std::vector<transmission_interface::TransmissionInfo> transmission_info;
    if (parser.parse(robot_description, transmission_info))
    {
      for (transmission_interface::TransmissionInfo info : transmission_info)
      {
        for (transmission_interface::JointInfo joint_info : info.joints_)
        {
          if (joint_info.name_ == joint_name_)
          {
            std::string interface = joint_info.hardware_interfaces_.front();
            if (interface == "hardware_interface/PositionJointInterface")
            {
              command_mode_ = "position";
            }
            else if (interface == "hardware_interface/VelocityJointInterface")
            {
              command_mode_ = "velocity";
            }
            else if (interface == "hardware_interface/EffortJointInterface")
            {
              command_mode_ = "effort";
            }
          }
        }
      }
    }
  }

  // initializes commands and states
  command_ = 0.0;
  position_ = 0.0;
  velocity_ = 0.0;
  effort_ = 0.0;

  initialize_ = true;
  position_steps_ = 0.0;

  // reads system parameters
  nh.param<double>("gear_ratio", gear_ratio_, 1.0);
  nh.param<double>("torque_const", torque_const_, 1.0);
  nh.param<int>("num_hall_sensors", num_hall_sensors_, 3);
  nh.param<double>("screw_lead", screw_lead_, 1.0);
  ROS_INFO("Gear ratio is set to %f", gear_ratio_);
  ROS_INFO("Torque constant is set to %f", torque_const_);
  ROS_INFO("The number of hall sensors is set to %d", num_hall_sensors_);
  ROS_INFO("Screw lead is set to %f", screw_lead_);

  // check num of rotor poles
  nh.param<int>("num_rotor_poles", num_rotor_poles_, 2);
  ROS_INFO("The number of rotor poles is set to %d", num_rotor_poles_);
  if (num_rotor_poles_ % 2 != 0)
  {
    ROS_ERROR("There should be even number of rotor poles");
    ros::shutdown();
    return false;
  }

  // reads driving mode setting
  // - assigns an empty string if param. is not found
  nh.getParam("command_mode", command_mode_);
  ROS_INFO("mode: %s", command_mode_.data());

  // check joint type
  joint_type_ = urdf::Joint::UNKNOWN;
  nh.getParam("joint_type", joint_type);
  ROS_INFO("joint type: %s", joint_type.data());
  if (joint_type == "revolute")
  {
    joint_type_ = urdf::Joint::REVOLUTE;
  }
  else if (joint_type == "continuous")
  {
    joint_type_ = urdf::Joint::CONTINUOUS;
  }
  else if (joint_type == "prismatic")
  {
    joint_type_ = urdf::Joint::PRISMATIC;
  }
  if ((joint_type_ != urdf::Joint::REVOLUTE) && (joint_type_ != urdf::Joint::CONTINUOUS) &&
      (joint_type_ != urdf::Joint::PRISMATIC))
  {
    ROS_ERROR("Verify your joint type");
    ros::shutdown();
    return false;
  }

  nh.param<double>("control_rate", control_rate_, 50.0);
  control_timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &VescHwInterface::controlTimerCallback, this);
  ROS_INFO("Control rate is set to %f Hz", control_rate_);

  // registers a state handle and its interface
  hardware_interface::JointStateHandle state_handle(joint_name_, &position_, &velocity_, &effort_);
  joint_state_interface_.registerHandle(state_handle);
  registerInterface(&joint_state_interface_);

  // registers specified command handle and its interface
  if (command_mode_ == "position")
  {
    hardware_interface::JointHandle position_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_position_interface_.registerHandle(position_handle);
    registerInterface(&joint_position_interface_);

    joint_limits_interface::PositionJointSaturationHandle limit_handle(position_handle, joint_limits_);
    limit_position_interface_.registerHandle(limit_handle);

    // initializes the servo controller
    servo_controller_.init(nh, &vesc_interface_);
    servo_controller_.setControlRate(control_rate_);
  }
  else if (command_mode_ == "velocity" || command_mode_ == "velocity_duty")
  {
    hardware_interface::JointHandle velocity_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_velocity_interface_.registerHandle(velocity_handle);
    registerInterface(&joint_velocity_interface_);

    joint_limits_interface::VelocityJointSaturationHandle limit_handle(velocity_handle, joint_limits_);
    limit_velocity_interface_.registerHandle(limit_handle);
    if (command_mode_ == "velocity_duty")
    {
      wheel_controller_.init(nh, &vesc_interface_);
      wheel_controller_.setGearRatio(gear_ratio_);
      wheel_controller_.setControlRate(control_rate_);
      wheel_controller_.setRotorPoles(num_rotor_poles_);
      wheel_controller_.setHallSensors(num_hall_sensors_);
    }
  }
  else if (command_mode_ == "effort" || command_mode_ == "effort_duty")
  {
    hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_effort_interface_.registerHandle(effort_handle);
    registerInterface(&joint_effort_interface_);

    joint_limits_interface::EffortJointSaturationHandle limit_handle(effort_handle, joint_limits_);
    limit_effort_interface_.registerHandle(limit_handle);
  }
  else
  {
    ROS_ERROR("Verify your command mode setting");
    ros::shutdown();
    return false;
  }

  return true;
}

void VescHwInterface::read()
{
  position_ = vesc_position_;
  velocity_ = vesc_velocity_;
  effort_ = vesc_effort_;

  if (command_mode_ != "position" && command_mode_ != "velocity_duty")
  {
    vesc_interface_.requestState();
  }

  return;
}

void VescHwInterface::read(const ros::Time& time, const ros::Duration& period)
{
  read();
  return;
}

void VescHwInterface::write()
{
  // sends commands
  if (command_mode_ == "position")
  {
    limit_position_interface_.enforceLimits(getPeriod());
    vesc_command_ = command_;
  }
  else if (command_mode_ == "velocity")
  {
    limit_velocity_interface_.enforceLimits(getPeriod());

    // converts the velocity unit: rad/s or m/s -> rpm -> erpm
    const double command_rpm = command_ * 60.0 / 2.0 / M_PI / gear_ratio_;
    const double command_erpm = command_rpm * static_cast<double>(num_rotor_poles_) / 2;

    // sends a reference velocity command
    vesc_interface_.setSpeed(command_erpm);
  }
  else if (command_mode_ == "velocity_duty")
  {
    limit_velocity_interface_.enforceLimits(getPeriod());
    vesc_command_ = command_;
  }
  else if (command_mode_ == "effort")
  {
    limit_effort_interface_.enforceLimits(getPeriod());

    // converts the command unit: Nm or N -> A
    const double command_current = command_ * gear_ratio_ / torque_const_;

    // sends a reference current command
    vesc_interface_.setCurrent(command_current);
  }
  else if (command_mode_ == "effort_duty")
  {
    command_ = std::clamp(command_, -1.0, 1.0);

    // sends a  duty command
    vesc_interface_.setDutyCycle(command_);
  }
  return;
}

void VescHwInterface::write(const ros::Time& time, const ros::Duration& period)
{
  write();
  return;
}

void VescHwInterface::controlTimerCallback(const ros::TimerEvent& e)
{
  if (command_mode_ == "velocity_duty")
  {
    wheel_controller_.control(vesc_command_, position_steps_);
  }
  else if (command_mode_ == "position")
  {
    servo_controller_.control(vesc_command_, vesc_position_);
    if (servo_controller_.isCalibrating())
    {
      vesc_command_ = servo_controller_.getCalibratingPosition();
    }
  }
  vesc_interface_.requestState();
}

ros::Time VescHwInterface::getTime() const
{
  return ros::Time::now();
}

ros::Duration VescHwInterface::getPeriod() const
{
  return ros::Duration(0.01);
}

double VescHwInterface::getZeroPosition()
{
  if (command_mode_ == "position")
    return servo_controller_.getZeroPosition();
  return 0.0;
}

void VescHwInterface::packetCallback(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);

    const double current = values->getMotorCurrent();
    const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_rotor_poles_ / 2);
    const int steps = static_cast<int>(values->getPosition());

    if (initialize_)
    {
      prev_steps_ = steps;
      initialize_ = false;
    }
    position_steps_ += static_cast<double>(steps - prev_steps_);
    prev_steps_ = steps;

    vesc_position_ = position_steps_ / (num_hall_sensors_ * num_rotor_poles_) * gear_ratio_;  // unit: revolution
    vesc_velocity_ = velocity_rpm * gear_ratio_;                                              // unit: rpm
    vesc_effort_ = current * torque_const_ / gear_ratio_;                                     // unit: Nm or N

    switch (joint_type_)
    {
      case urdf::Joint::REVOLUTE:
      case urdf::Joint::CONTINUOUS:
        vesc_position_ = vesc_position_ * 2.0 * M_PI;         // unit: rad
        vesc_velocity_ = vesc_velocity_ / 60.0 * 2.0 * M_PI;  // unit: rad/s
        break;
      case urdf::Joint::PRISMATIC:
        vesc_position_ = vesc_position_ * screw_lead_;         // unit: m
        vesc_velocity_ = vesc_velocity_ / 60.0 * screw_lead_;  // unit: m/s
        break;
    }

    vesc_position_ -= getZeroPosition();
  }

  return;
}

void VescHwInterface::errorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
  return;
}

}  // namespace vesc_hw_interface

PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::VescHwInterface, hardware_interface::RobotHW)
