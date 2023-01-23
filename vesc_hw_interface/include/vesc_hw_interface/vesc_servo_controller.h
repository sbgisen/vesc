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

#ifndef VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_H_
#define VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <limits>

#include <angles/angles.h>
#include <ros/ros.h>
#include <urdf_model/joint.h>
#include <vesc_driver/vesc_interface.h>
#include <vesc_hw_interface/vesc_step_difference.h>

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

  void init(ros::NodeHandle, VescInterface*);
  void control();
  void setTargetPosition(const double position_reference);
  void setGearRatio(const double gear_ratio);
  void setTorqueConst(const double torque_const);
  void setRotorPoles(const int rotor_poles);
  void setHallSensors(const int hall_sensors);
  void setJointType(const int joint_type);
  void setScrewLead(const double screw_lead);
  double getZeroPosition() const;
  double getPositionSens(void);
  double getVelocitySens(void);
  double getEffortSens(void);
  void executeCalibration();
  void updateSensor(const std::shared_ptr<VescPacket const>&);

private:
  VescInterface* interface_ptr_;
  VescStepDifference vesc_step_difference_;

  const std::string DUTY = "duty";
  const std::string CURRENT = "current";

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
  bool enable_speed_limit_;
  double speed_max_;
  ros::Timer control_timer_;
  // Internal variables for PID control
  double target_pose_;
  double target_pose_limited_;
  double target_pose_previous_;
  double sens_pose_, sens_vel_, sens_eff_;
  double sens_pose_previous_;
  double position_steps_;
  int16_t steps_previous_;
  double error_integ_;
  // Internal variables for initialization
  bool sensor_initialize_;
  int calibration_steps_;
  double calibration_previous_position_;

  bool calibrate(const double);
  void limitTargetSpeed(void);
  void controlTimerCallback(const ros::TimerEvent& e);
};

}  // namespace vesc_hw_interface

#endif  // VESC_HW_INTERFACE_VESC_SERVO_CONTROLLER_H_
