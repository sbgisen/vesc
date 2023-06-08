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

#include "vesc_hw_interface/vesc_servo_controller.h"
#include <exception>
#include <fstream>
#include "serial/serial.h"

namespace vesc_hw_interface
{
VescServoController::VescServoController() : gear_ratio_(1.0), torque_const_(1.0), num_rotor_poles_(1)
{
}

VescServoController::~VescServoController()
{
  interface_ptr_->setDutyCycle(0.0);
  serial_.close();
}

void VescServoController::init(ros::NodeHandle nh, VescInterface* interface_ptr, const double gear_ratio,
                               const double torque_const, const int rotor_poles, const int hall_sensors,
                               const int joint_type, const double screw_lead)
{
  // initializes members
  if (interface_ptr == NULL)
  {
    ros::shutdown();
  }
  else
  {
    interface_ptr_ = interface_ptr;
  }

  gear_ratio_ = gear_ratio;
  torque_const_ = torque_const;
  num_rotor_poles_ = rotor_poles;
  num_hall_sensors_ = hall_sensors;
  joint_type_ = joint_type;
  screw_lead_ = screw_lead;

  calibration_flag_ = true;
  sensor_initialize_ = true;
  zero_position_ = 0.0;
  error_integ_ = 0.0;
  steps_previous_ = 0;
  position_steps_ = 0;
  calibration_steps_ = 0;
  calibration_previous_position_ = 0.0;

  // reads parameters
  nh.param<double>("servo/Kp", kp_, 50.0);
  nh.param<double>("servo/Ki", ki_, 0.0);
  nh.param<double>("servo/Kd", kd_, 1.0);
  nh.param<double>("servo/i_clamp", i_clamp_, 1.0);
  nh.param<double>("servo/duty_limiter", duty_limiter_, 1.0);
  nh.param<bool>("servo/antiwindup", antiwindup_, true);
  nh.param<double>("servo/control_rate", control_rate_, 100.0);
  nh.param<double>("servo/calibration_current", calibration_current_, 6.0);
  nh.param<double>("servo/calibration_duty", calibration_duty_, 0.1);
  nh.param<std::string>("servo/calibration_mode", calibration_mode_, "current");
  nh.param<double>("servo/calibration_position", calibration_position_, 0.0);
  nh.param<bool>("servo/calibration", calibration_flag_, true);
  nh.param<std::string>("servo/calibration_result_path", calibration_result_path_, "");
  nh.param<bool>("servo/enable_sensor_calibration", enable_sensor_calibration_, false);
  nh.param<std::string>("servo/calibration_port", calibration_port_, "/dev/ttyUSB0");
  nh.param<int>("servo/calibration_baudrate", calibration_baudrate_, 9600);

  if (!calibration_result_path_.empty())
  {
    ROS_INFO("[Servo Control] Latest position will be saved to %s", calibration_result_path_.data());
  }
  if (!calibration_flag_)
  {
    if (!nh.hasParam("servo/last_position"))
    {
      ROS_ERROR("[Servo Control] No last_position parameter found. Please calibrate servo first.");
      ros::shutdown();
    }
    nh.param<double>("servo/last_position", target_position_, 0.0);
  }

  // shows parameters
  ROS_INFO("[Servo Gains] P: %f, I: %f, D: %f", kp_, ki_, kd_);
  if (calibration_mode_ == CURRENT)
  {
    ROS_INFO("[Servo Calibration] Mode: %s, value: %f", CURRENT.data(), calibration_current_);
  }
  else if (calibration_mode_ == DUTY)
  {
    ROS_INFO("[Servo Calibration] Mode: %s, value: %f", DUTY.data(), calibration_duty_);
  }
  else
  {
    ROS_ERROR("[Servo Calibration] Invalid mode");
  }

  // Smoothing differentiation when hall sensor resolution is insufficient
  bool smooth_diff;
  nh.param<bool>("servo/enable_smooth_diff", smooth_diff, true);
  if (smooth_diff)
  {
    double smooth_diff_max_sampling_time;
    int counter_td_vw_max_step;
    nh.param<double>("servo/smooth_diff/max_sample_sec", smooth_diff_max_sampling_time, 1.0);
    nh.param<int>("servo/smooth_diff/max_smooth_step", counter_td_vw_max_step, 10);
    vesc_step_difference_.enableSmooth(control_rate_, smooth_diff_max_sampling_time, counter_td_vw_max_step);
    ROS_INFO("[Servo Control] Smooth differentiation enabled, max_sample_sec: %f, max_smooth_step: %d",
             smooth_diff_max_sampling_time, counter_td_vw_max_step);
  }

  // Restore last position
  if (!calibration_flag_)
  {
    target_position_previous_ = target_position_;
    sens_position_ = target_position_;

    position_steps_ = sens_position_ * (num_hall_sensors_ * num_rotor_poles_) / gear_ratio_;

    switch (joint_type_)
    {
      case urdf::Joint::REVOLUTE:
      case urdf::Joint::CONTINUOUS:
        position_steps_ /= 2.0 * M_PI;
        break;
      case urdf::Joint::PRISMATIC:
        position_steps_ /= screw_lead_;
        break;
    }
    vesc_step_difference_.resetStepDifference(position_steps_);
  }

  // Create timer callback for PID servo control
  control_timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &VescServoController::controlTimerCallback, this);

  // Open port to calibration sensor
  if (enable_sensor_calibration_)
  {
    try
    {
      serial_.setPort(calibration_port_);
      serial_.setBaudrate(calibration_baudrate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serial_.setTimeout(to);
      serial_.open();
      if (!serial_.isOpen())
      {
        throw serial::SerialException("Unable to open serial port");
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR("[Servo Control] Failed to open serial port %s, use manual calibration instead",
                calibration_port_.data());
      enable_sensor_calibration_ = false;
    }
    ROS_INFO("[Servo Control] Serial port %s opened", calibration_port_.data());
  }
  return;
}

void VescServoController::control()
{
  if (sensor_initialize_)
    return;
  // executes calibration
  if (calibration_flag_)
  {
    calibrate();
    // initializes/resets control variables
    target_position_previous_ = calibration_position_;
    vesc_step_difference_.resetStepDifference(position_steps_);
    return;
  }

  // PID control
  double step_diff = vesc_step_difference_.getStepDifference(position_steps_);
  double current_vel = step_diff * 2.0 * M_PI / (num_rotor_poles_ * num_hall_sensors_) * control_rate_ * gear_ratio_;
  double target_vel = (target_position_ - target_position_previous_) * control_rate_;

  double error = target_position_ - sens_position_;
  double error_dt = target_vel - current_vel;
  double error_integ_prev = error_integ_;
  error_integ_ += (error / control_rate_);
  error_integ_ = std::clamp(error_integ_, -i_clamp_ / ki_, i_clamp_ / ki_);

  const double u_p = kp_ * error;
  const double u_d = kd_ * error_dt;
  const double u_i = ki_ * error_integ_;
  double u = u_p + u_d + u_i;

  // limit integral
  if (antiwindup_)
  {
    if (u > duty_limiter_ && error_integ_ > 0)
    {
      error_integ_ = std::max(0.0, (duty_limiter_ - u_p - u_d) / ki_);
    }
    else if (u < -duty_limiter_ && error_integ_ < 0)
    {
      error_integ_ = std::min(0.0, (-duty_limiter_ - u_p - u_d) / ki_);
    }
  }

  // limit duty value
  u = std::clamp(u, -duty_limiter_, duty_limiter_);

  // updates previous data
  target_position_previous_ = target_position_;

  // command duty
  interface_ptr_->setDutyCycle(u);
  return;
}

void VescServoController::setTargetPosition(const double position)
{
  target_position_ = position;
}

void VescServoController::setGearRatio(const double gear_ratio)
{
  gear_ratio_ = gear_ratio;
  ROS_INFO("[VescServoController]Gear ratio is set to %f", gear_ratio_);
}

void VescServoController::setTorqueConst(const double torque_const)
{
  torque_const_ = torque_const;
  ROS_INFO("[VescServoController]Torque constant is set to %f", torque_const_);
}

void VescServoController::setRotorPoles(const int rotor_poles)
{
  num_rotor_poles_ = rotor_poles;
  ROS_INFO("[VescServoController]The number of rotor pole is set to %d", num_rotor_poles_);
}

void VescServoController::setHallSensors(const int hall_sensors)
{
  num_hall_sensors_ = hall_sensors;
  ROS_INFO("[VescServoController]The number of hall sensors is set to %d", num_hall_sensors_);
}

void VescServoController::setJointType(const int joint_type)
{
  joint_type_ = joint_type;
}

void VescServoController::setScrewLead(const double screw_lead)
{
  screw_lead_ = screw_lead;
  ROS_INFO("[VescServoController]Screw lead is set to %f", screw_lead_);
}

double VescServoController::getZeroPosition() const
{
  return zero_position_;
}

double VescServoController::getPositionSens()
{
  if (calibration_flag_)
  {
    return calibration_position_;
  }
  return sens_position_;
}

double VescServoController::getVelocitySens()
{
  return sens_velocity_;
}

double VescServoController::getEffortSens()
{
  return sens_effort_;
}

void VescServoController::executeCalibration()
{
  calibration_flag_ = true;
  return;
}

bool VescServoController::calibrate()
{
  // sends a command for calibration
  if (calibration_mode_ == CURRENT)
  {
    interface_ptr_->setCurrent(calibration_current_);
  }
  else if (calibration_mode_ == DUTY)
  {
    interface_ptr_->setDutyCycle(calibration_duty_);
  }
  else
  {
    ROS_ERROR("Please set the calibration mode surely");
    return false;
  }

  calibration_steps_++;

  if (enable_sensor_calibration_)
  {
    std::string data = serial_.read(serial_.available());
    for (auto detected : data)
    {
      if (static_cast<bool>(detected))
      {
        // finishes calibrating
        calibration_steps_ = 0;
        zero_position_ = sens_position_ - calibration_position_;
        target_position_ = calibration_position_;
        ROS_INFO("Calibration Finished");
        calibration_flag_ = false;
        serial_.close();
        return true;
      }
    }
    return false;
  }
  else if (calibration_steps_ % 20 == 0)
  {
    if (std::abs(sens_position_ - calibration_previous_position_) <= std::numeric_limits<double>::epsilon())
    {
      // finishes calibrating
      calibration_steps_ = 0;
      zero_position_ = sens_position_ - calibration_position_;
      target_position_ = calibration_position_;
      ROS_INFO("Calibration Finished");
      calibration_flag_ = false;
      return true;
    }
    else
    {
      calibration_previous_position_ = sens_position_;
      return false;
    }
  }
  else
  {
    // continues calibration
    return false;
  }
}

void VescServoController::controlTimerCallback(const ros::TimerEvent& e)
{
  control();
  interface_ptr_->requestState();
}

void VescServoController::updateSensor(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);
    const double current = values->getMotorCurrent();
    const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_rotor_poles_ / 2);
    const int32_t steps = static_cast<int32_t>(values->getPosition());
    if (sensor_initialize_)
    {
      steps_previous_ = steps;
      sensor_initialize_ = false;
    }
    const int32_t steps_diff = steps - steps_previous_;
    position_steps_ += static_cast<double>(steps_diff);
    steps_previous_ = steps;

    sens_position_ = position_steps_ / (num_hall_sensors_ * num_rotor_poles_) * gear_ratio_;  // unit: revolution
    sens_velocity_ = velocity_rpm * gear_ratio_;                                              // unit: rpm
    sens_effort_ = current * torque_const_ / gear_ratio_;

    switch (joint_type_)
    {
      case urdf::Joint::REVOLUTE:
      case urdf::Joint::CONTINUOUS:
        sens_position_ = sens_position_ * 2.0 * M_PI;         // unit: rad
        sens_velocity_ = sens_velocity_ / 60.0 * 2.0 * M_PI;  // unit: rad/s
        break;
      case urdf::Joint::PRISMATIC:
        sens_position_ = sens_position_ * screw_lead_;         // unit: m
        sens_velocity_ = sens_velocity_ / 60.0 * screw_lead_;  // unit: m/s
        break;
    }

    sens_position_ -= getZeroPosition();

    if (!calibration_result_path_.empty())
    {
      std::ofstream file;
      file.open(calibration_result_path_, std::ios::out);
      file << "servo/last_position: " << sens_position_ << std::endl;
      file.close();
    }
  }
  return;
}
}  // namespace vesc_hw_interface
