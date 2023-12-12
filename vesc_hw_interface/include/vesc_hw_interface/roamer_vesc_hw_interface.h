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

#ifndef XR1_HW_INTERFACE_VESC_HW_INTERFACE_H_
#define XR1_HW_INTERFACE_VESC_HW_INTERFACE_H_

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <angles/angles.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <urdf_model/types.h>
#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.hpp>

#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_packet_factory.h"
#include "vesc_hw_interface/vesc_servo_controller.h"
#include "vesc_hw_interface/vesc_wheel_controller.h"

namespace vesc_hw_interface
{
using vesc_driver::VescInterface;
using vesc_driver::VescPacket;
using vesc_driver::VescPacketValues;

class XR1VescHwInterface : public hardware_interface::RobotHW
{
public:
  XR1VescHwInterface();
  ~XR1VescHwInterface();

  bool init(ros::NodeHandle&, ros::NodeHandle&);
  void read(const ros::Time&, const ros::Duration&);
  void write(const ros::Time&, const ros::Duration&);
  ros::Time getTime() const;

private:
  VescInterface vesc_interface_;
  VescServoController servo_controller_;
  VescWheelController wheel_controller_;

  std::string joint_name_, command_mode_;
  int joint_type_;

  // double command_;
  // double position_, velocity_, effort_;  // joint states
  // double command2_;
  // double position2_, velocity2_, effort2_;  // joint states

  // motors
  double cmds_[4];            // 4 actuators: (0)front-left, (1)rear-left, (2)front-right, (3)rear-right
  double p_wheel_pos_[4];     // powered wheels position, indexing same as cmds_[]
  double p_wheel_vel_[4];     // powered wheels velocity, indexing same as cmds_[]
  double p_wheel_eff_[4];     // powered wheels effort, indexing same as cmds_[]

  // rocker and bogie sensors
  double rb_pos_[4];        // rocker position sensor: (0)rocker left, (1)bogie left, (2)rocker right, (3)bogie right
  double rb_vel_[4];        // rocker velocity sensor
  double rb_eff_[4];        // rocker effort sensor

  int num_rotor_poles_;               // the number of rotor poles
  int num_hall_sensors_;              // the number of hall sensors
  double gear_ratio_, torque_const_;  // physical params
  double screw_lead_;                 // linear distance (m) of 1 revolution

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  joint_limits_interface::JointLimits joint_limits_;
  joint_limits_interface::PositionJointSaturationInterface limit_position_interface_;
  joint_limits_interface::VelocityJointSaturationInterface limit_velocity_interface_;
  joint_limits_interface::EffortJointSaturationInterface limit_effort_interface_;

  void packetCallback(const std::shared_ptr<VescPacket const>&);
  void errorCallback(const std::string&);

  void makeWheelVelInterface(const std::string &joint_name, int cmd_index, const ros::NodeHandle &nh);

  /**
   * @brief Creates a mock hw velocity + state interface with indeces:
   * 0: wheel_front_left_joint
   * 1: wheel_rear_left_joint
   * 2: wheel_front_right_joint
   * 3: wheel_rear_right_joint
   * 
   * @param joint_name 
   * @param cmd_index 
   */
  void makeMockVelInterface(const std::string &joint_name, int cmd_index);
  
  /**
   * @brief Creates a mock hw state interface with indeces:
   * 0: rocker_left_joint
   * 1: bogie_left_joint
   * 2: rocker_right_joint
   * 3: bogie_right_joint
   * 
   * @param joint_name 
   * @param cmd_index 
   */
  void makeMockStateInterface(const std::string &joint_name, int cmd_index);

  // struct Joint
  // {
  //   double position;
  //   double position_offset;
  //   double velocity;
  //   double effort;
  //   double cmd;

  //   Joint() :
  //     position(0), velocity(0), effort(0), cmd(0)
  //   { }
  // }
  // joints_[4];
};

}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_HW_INTERFACE_H_
