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

namespace vesc_hw_interface
{
VescServoController::VescServoController()
{
}

VescServoController::~VescServoController()
{
  interface_ptr_->setDutyCycle(0.0);
}

void VescServoController::init(ros::NodeHandle nh, VescInterface* interface_ptr)
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

  calibration_flag_ = true;
  zero_position_ = 0.0;
  error_integ_ = 0.0;
  calibration_steps_ = 0;
  calibration_previous_position_ = 0.0;

  // reads parameters
  nh.param("servo/Kp", Kp_, 1.0);
  nh.param("servo/Ki", Ki_, 0.0);
  nh.param("servo/Kd", Kd_, 0.0);
  nh.param("servo/calibration_current", calibration_current_, 6.0);
  nh.param("servo/calibration_duty", calibration_duty_, 0.1);
  nh.param<std::string>("servo/calibration_mode", calibration_mode_, "current");
  nh.param("servo/calibration_position", calibration_position_, 0.0);
  nh.param("servo/speed_limit", speed_limit_, 1.0);

  // shows parameters
  ROS_INFO("[Servo Gains] P: %f, I: %f, D: %f", Kp_, Ki_, Kd_);
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
  return;
}

void VescServoController::control(const double position_reference, const double position_current)
{
  double position_target = updateSpeedLimitedPositionReference(position_reference);

  // executes calibration
  if (calibration_flag_)
  {
    calibrate(position_current);
    // initializes/resets control variables
    time_previous_ = ros::Time::now();
    position_sens_previous_ = position_current;
    position_reference_previous_ = calibration_position_;
    error_previous_ = 0.0;
    return;
  }

  const ros::Time time_current = ros::Time::now();
  // calculates PD control
  const double error_current = position_target - position_current;
  const double u_pd = Kp_ * error_current + Kd_ * (error_current - error_previous_) / control_period_;

  double u = 0.0;

  // calculates I control if PD input is not saturated
  if (isSaturated(u_pd))
  {
    u = saturate(u_pd);
  }
  else
  {
    double error_integ_new = error_integ_ + (error_current + error_previous_) / 2.0 * control_period_;
    const double u_pid = u_pd + Ki_ * error_integ_new;

    // not use I control if PID input is saturated
    // since error integration causes bugs
    if (isSaturated(u_pid))
    {
      u = saturate(u_pid);
    }
    else
    {
      u = u_pid;
      error_integ_ = error_integ_new;
    }
  }

  // updates previous data
  error_previous_ = error_current;
  time_previous_ = time_current;
  position_sens_previous_ = position_current;
  position_reference_previous_ = position_target;

  // command duty
  interface_ptr_->setDutyCycle(u);
  return;
}

void VescServoController::setControlRate(const double control_rate)
{
  control_rate_ = control_rate;
  control_period_ = 1 / control_rate;
}

bool VescServoController::isCalibrating()
{
  return calibration_flag_;
}

double VescServoController::getCalibratingPosition() const
{
  return calibration_position_;
}

double VescServoController::getZeroPosition() const
{
  return zero_position_;
}

void VescServoController::executeCalibration()
{
  calibration_flag_ = true;
  return;
}

bool VescServoController::calibrate(const double position_current)
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

  if (calibration_steps_ % 20 == 0)
  {
    if (std::abs(position_current - calibration_previous_position_) <= std::numeric_limits<double>::epsilon())
    {
      // finishes calibrating
      calibration_steps_ = 0;
      zero_position_ = position_current - calibration_position_;
      ROS_INFO("Calibration Finished");
      calibration_flag_ = false;
      return true;
    }
    else
    {
      calibration_previous_position_ = position_current;
      return false;
    }
  }
  else
  {
    // continues calibration
    return false;
  }
}

bool VescServoController::isSaturated(const double arg) const
{
  return std::abs(arg) > 1.0;
}

double VescServoController::saturate(const double arg) const
{
  return std::clamp(arg, -1.0, 1.0);
}

double VescServoController::updateSpeedLimitedPositionReference(double position_target)
{
  if (position_target > (position_reference_previous_ + speed_limit_ * control_period_))
  {
    return position_reference_previous_ + speed_limit_ * control_period_;
  }
  else if (position_target < (position_reference_previous_ - speed_limit_ * control_period_))
  {
    return position_reference_previous_ - speed_limit_ * control_period_;
  }
  else
  {
    return position_target;
  }
}

}  // namespace vesc_hw_interface
