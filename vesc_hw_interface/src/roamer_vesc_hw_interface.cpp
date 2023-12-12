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

#include "vesc_hw_interface/roamer_vesc_hw_interface.h"

namespace vesc_hw_interface
{
XR1VescHwInterface::XR1VescHwInterface()
  : vesc_interface_(std::string(), std::bind(&XR1VescHwInterface::packetCallback, this, std::placeholders::_1),
                    std::bind(&XR1VescHwInterface::errorCallback, this, std::placeholders::_1))
{
}

XR1VescHwInterface::~XR1VescHwInterface()
{
}

bool XR1VescHwInterface::init(ros::NodeHandle& nh_root, ros::NodeHandle& nh)
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

  // // loads joint limits
  // std::string robot_description_name, robot_description;
  // nh.param<std::string>("robot_description_name", robot_description_name, "/robot_description");

  // // parses the urdf and extract joint type
  // std::string joint_type;
  // if (nh.getParam(robot_description_name, robot_description))
  // {
  //   const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(robot_description);
  //   const urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(joint_name_);

  //   if (getJointLimits(urdf_joint, joint_limits_))
  //   {
  //     ROS_INFO("Joint limits are loaded");
  //     // fprintf(stderr, "pos_lim: [%d, %f, %f]\n", joint_limits_.has_position_limits, joint_limits_.min_position, joint_limits_.max_position);
  //     // fprintf(stderr, "vel_lim: [%d, %f]\n", joint_limits_.has_velocity_limits, joint_limits_.max_velocity);
  //     // fprintf(stderr, "eff_lim: [%d, %f]\n", joint_limits_.has_effort_limits, joint_limits_.max_effort);
  //     // fprintf(stderr, "acc_lim: [%d, %f]\n", joint_limits_.has_acceleration_limits, joint_limits_.max_acceleration);
  //     // fprintf(stderr, "jrk_lim: [%d, %f]\n", joint_limits_.has_jerk_limits, joint_limits_.max_jerk);
  //     // fprintf(stderr, "ang_WA: [%d]\n", joint_limits_.angle_wraparound);
  //   }
    
  //   switch (urdf_joint->type)
  //   {
  //     case urdf::Joint::REVOLUTE:
  //       joint_type = "revolute";
  //       break;
  //     case urdf::Joint::CONTINUOUS:
  //       joint_type = "continuous";
  //       break;
  //     case urdf::Joint::PRISMATIC:
  //       joint_type = "prismatic";
  //       break;
  //   }
  //   // ROS_INFO("RANDEL: Joint: [%s] is (%s)", joint_name_.c_str(), joint_type.c_str());
  // }

  // **** RANDEL: hardcode limits instead of parsing from URDF so that we can run this BEFORE URDF!!!!
  std::string joint_type("continuous");
  joint_limits_.has_velocity_limits = true;
  joint_limits_.max_velocity = 1000.0;
  joint_limits_.has_effort_limits = true;
  joint_limits_.max_effort = 100.0;

  // initializes commands and states
  // command_ = 0.0;
  // position_ = 0.0;
  // velocity_ = 0.0;
  // effort_ = 0.0;

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
  nh.param<std::string>("command_mode", command_mode_, "");
  ROS_INFO("mode: %s", command_mode_.data());

  // check joint type
  joint_type_ = urdf::Joint::UNKNOWN;
  // nh.getParam("joint_type", joint_type);
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

  // registers a state handle and its interface
  hardware_interface::JointStateHandle state_handle(joint_name_, &p_wheel_pos_[0], &p_wheel_vel_[0], &p_wheel_eff_[0]);
  joint_state_interface_.registerHandle(state_handle);
  registerInterface(&joint_state_interface_);

  // registers specified command handle and its interface
  if (command_mode_ == "velocity" || command_mode_ == "velocity_duty")
  {
    hardware_interface::JointHandle velocity_handle(joint_state_interface_.getHandle(joint_name_), &cmds_[0]);
    joint_velocity_interface_.registerHandle(velocity_handle);
    registerInterface(&joint_velocity_interface_);

    joint_limits_interface::VelocityJointSaturationHandle limit_handle(velocity_handle, joint_limits_);
    limit_velocity_interface_.registerHandle(limit_handle);
    if (command_mode_ == "velocity_duty")
    {
      wheel_controller_.init(nh, &vesc_interface_);
      wheel_controller_.setGearRatio(gear_ratio_);
      wheel_controller_.setTorqueConst(torque_const_);
      wheel_controller_.setRotorPoles(num_rotor_poles_);
      wheel_controller_.setHallSensors(num_hall_sensors_);
    }
  }
  else if (command_mode_ == "effort" || command_mode_ == "effort_duty")
  {
    hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_name_), &cmds_[0]);
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


  // ***************************************************
  // **** rocker and bogie *****************************
  // reset
  memset(cmds_, 0, sizeof(cmds_));
  memset(p_wheel_pos_, 0, sizeof(p_wheel_pos_));
  memset(p_wheel_vel_, 0, sizeof(p_wheel_vel_));
  memset(p_wheel_eff_, 0, sizeof(p_wheel_eff_));
  memset(rb_pos_, 0, sizeof(rb_pos_));
  memset(rb_vel_, 0, sizeof(rb_vel_));
  memset(rb_eff_, 0, sizeof(rb_eff_));


  // RANDEL: for mock hw interface
  makeMockVelInterface(std::string("wheel_rear_left_joint"), 1);
  makeMockVelInterface(std::string("wheel_front_right_joint"), 2);
  makeMockVelInterface(std::string("wheel_rear_right_joint"), 3);

  makeMockVelInterface(std::string("wheel_middle_left_joint"), 0);
  makeMockVelInterface(std::string("wheel_middle_right_joint"), 2);

  makeMockStateInterface(std::string("rocker_left_joint"), 0);
  makeMockStateInterface(std::string("bogie_left_joint"), 1);
  makeMockStateInterface(std::string("rocker_right_joint"), 2);
  makeMockStateInterface(std::string("bogie_right_joint"), 3);


  return true;
}


void XR1VescHwInterface::makeWheelVelInterface(const std::string &joint_name, int cmd_index, const ros::NodeHandle &nh){
  if (command_mode_ == "velocity" || command_mode_ == "velocity_duty")
  {
    hardware_interface::JointHandle velocity_handle(joint_state_interface_.getHandle(joint_name.c_str()), &cmds_[cmd_index]);
    joint_velocity_interface_.registerHandle(velocity_handle);
    registerInterface(&joint_velocity_interface_);

    joint_limits_interface::VelocityJointSaturationHandle limit_handle(velocity_handle, joint_limits_);
    limit_velocity_interface_.registerHandle(limit_handle);
    if (command_mode_ == "velocity_duty")
    {
      wheel_controller_.init(nh, &vesc_interface_);
      wheel_controller_.setGearRatio(gear_ratio_);
      wheel_controller_.setTorqueConst(torque_const_);
      wheel_controller_.setRotorPoles(num_rotor_poles_);
      wheel_controller_.setHallSensors(num_hall_sensors_);
    }
  }
}

void XR1VescHwInterface::makeMockVelInterface(const std::string &joint_name, int cmd_index){
  hardware_interface::JointStateHandle _state_handle(joint_name.c_str(), &p_wheel_pos_[cmd_index], &p_wheel_vel_[cmd_index], &p_wheel_eff_[cmd_index]);
  joint_state_interface_.registerHandle(_state_handle);
  registerInterface(&joint_state_interface_);

  hardware_interface::JointHandle _vel_handle(joint_state_interface_.getHandle(joint_name.c_str()), &cmds_[cmd_index]);
  joint_velocity_interface_.registerHandle(_vel_handle);
  registerInterface(&joint_velocity_interface_);
}

void XR1VescHwInterface::makeMockStateInterface(const std::string &joint_name, int cmd_index){
  hardware_interface::JointStateHandle _state_handle(joint_name.c_str(), &rb_pos_[cmd_index], &rb_vel_[cmd_index], &rb_eff_[cmd_index]);
  joint_state_interface_.registerHandle(_state_handle);
  registerInterface(&joint_state_interface_);

}

void XR1VescHwInterface::read(const ros::Time& time, const ros::Duration& period)
{
  // requests joint states
  // function `packetCallback` will be called after receiving return packets
  // ROS_INFO("***RANDEL: Readi9ng");
  /*if (command_mode_ == "position")
  {
    // For PID control, request packets are automatically sent in the control cycle.
    // The latest data is read in this function.
    position_ = servo_controller_.getPositionSens();
    velocity_ = servo_controller_.getVelocitySens();
    effort_ = servo_controller_.getEffortSens();
  }
  else*/ if (command_mode_ == "velocity_duty")
  {
    p_wheel_pos_[0] = wheel_controller_.getPositionSens();
    p_wheel_vel_[0] = wheel_controller_.getVelocitySens();
    p_wheel_eff_[0] = wheel_controller_.getEffortSens();
  }
  else
  {
    // ROS_INFO("***RANDEL: requesting state!");
    vesc_interface_.requestState();
  }

  // ROS_INFO("***RANDEL: joint: %d", joint_type_);
  if (joint_type_ == urdf::Joint::REVOLUTE || joint_type_ == urdf::Joint::CONTINUOUS)
  {
    p_wheel_pos_[0] = angles::normalize_angle(p_wheel_pos_[0]);
    // ROS_INFO("***RANDEL: uuuuuu %f", position_);
  }

  return;
}

void XR1VescHwInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // sends commands
  if (command_mode_ == "position")
  {
    // Limit the speed using the parameters listed in xacro
    limit_position_interface_.enforceLimits(period);

    // executes PID control
    servo_controller_.setTargetPosition(cmds_[0]);
  }
  else if (command_mode_ == "velocity")
  {
    limit_velocity_interface_.enforceLimits(period);

    // converts the velocity unit: rad/s or m/s -> rpm -> erpm
    const double command_rpm = cmds_[0] * 60.0 / 2.0 / M_PI / gear_ratio_;
    const double command_erpm = command_rpm * static_cast<double>(num_rotor_poles_) / 2;


    // fprintf(stderr, "****CMDDDD: %f, %f\n", command_, command_erpm);
    // sends a reference velocity command
    vesc_interface_.setSpeed(command_erpm);
  }
  else if (command_mode_ == "velocity_duty")
  {
    limit_velocity_interface_.enforceLimits(period);

    // executes PID control
    wheel_controller_.setTargetVelocity(cmds_[0]);
  }
  else if (command_mode_ == "effort")
  {
    limit_effort_interface_.enforceLimits(period);

    // converts the command unit: Nm or N -> A
    const double command_current = cmds_[0] * gear_ratio_ / torque_const_;

    // sends a reference current command
    vesc_interface_.setCurrent(command_current);
  }
  else if (command_mode_ == "effort_duty")
  {
    cmds_[0] = std::max(-1.0, cmds_[0]);
    cmds_[0] = std::min(1.0, cmds_[0]);

    // sends a  duty command
    vesc_interface_.setDutyCycle(cmds_[0]);
  }
  return;
}

ros::Time XR1VescHwInterface::getTime() const
{
  return ros::Time::now();
}

void XR1VescHwInterface::packetCallback(const std::shared_ptr<VescPacket const>& packet)
{
  // ROS_INFO("***RANDEL: packetCallback CALLED!!!");
  if (!vesc_interface_.isRxDataUpdated())
  {
    ROS_WARN("[XR1VescHwInterface::packetCallback]packetCallcack called, but no packet received");
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

    p_wheel_pos_[0] = steps / (num_hall_sensors_ * num_rotor_poles_) * gear_ratio_;  // unit: rad or m
    p_wheel_vel_[0] = velocity_rpm / 60.0 * 2.0 * M_PI * gear_ratio_;                // unit: rad/s or m/s
    p_wheel_eff_[0] = current * torque_const_ / gear_ratio_;                           // unit: Nm or N
  }

  return;
}

void XR1VescHwInterface::errorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
  return;
}

}  // namespace vesc_hw_interface

PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::XR1VescHwInterface, hardware_interface::RobotHW)
