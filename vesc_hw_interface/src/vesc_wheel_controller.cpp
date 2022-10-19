/*********************************************************************
 * Copyright (c) 2022 SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************************************************************/

#include "vesc_hw_interface/vesc_wheel_controller.h"

namespace vesc_hw_interface
{
void VescWheelController::init(ros::NodeHandle nh, VescInterface* interface_ptr)
{
  if (interface_ptr == NULL)
  {
    ros::shutdown();
  }
  else
  {
    interface_ptr_ = interface_ptr;
  }

  nh.param<double>("motor/Kp", kp_, 0.005);
  nh.param<double>("motor/Ki", ki_, 0.005);
  nh.param<double>("motor/Kd", kd_, 0.0025);
  nh.param<double>("motor/i_clamp", i_clamp_, 0.2);
  nh.param<double>("motor/duty_limiter", duty_limiter_, 1.0);
  nh.param<bool>("motor/antiwindup", antiwindup_, true);
  ROS_INFO("motor/Kp: %f", kp_);
  ROS_INFO("motor/Ki: %f", ki_);
  ROS_INFO("motor/Kd: %f", kd_);
  ROS_INFO("motor/i_clamp: %f", i_clamp_);
  ROS_INFO("motor/duty_limiter: %f", duty_limiter_);
  ROS_INFO("antiwindup: %s", antiwindup_ ? "true" : "false");

  nh.param<int>("num_motor_pole_pairs", num_motor_pole_pairs_, 1.0);
  ctrl_frequency_ = 50.0;
}

void VescWheelController::control(const double target_velocity, const double current_pulse, bool initialize)
{
  const double motor_hall_ppr = static_cast<double>(num_motor_pole_pairs_);
  const double count_deviation_limit = static_cast<double>(num_motor_pole_pairs_);

  if (initialize)
  {
    target_pulse_ = current_pulse;
    error_ = 0.0;
    error_dt_ = 0.0;
    error_integ_ = 0.0;
    error_integ_prev_ = 0.0;
    this->counterTD(current_pulse, true);
  }
  else
  {  // convert rad/s to pulse
    target_pulse_ += target_velocity * motor_hall_ppr / (2 * M_PI) / ctrl_frequency_;
  }

  // overflow check
  if (target_pulse_ > static_cast<double>(LONG_MAX))
  {
    target_pulse_ += static_cast<double>(LONG_MIN);
  }
  else if (target_pulse_ < static_cast<double>(LONG_MIN))
  {
    target_pulse_ += static_cast<double>(LONG_MAX);
  }

  // limit error
  if (target_pulse_ - current_pulse > count_deviation_limit)
  {
    target_pulse_ = current_pulse + count_deviation_limit;
  }
  else if (target_pulse_ - current_pulse < -count_deviation_limit)
  {
    target_pulse_ = current_pulse - count_deviation_limit;
  }

  // pid control
  error_dt_ = target_velocity - (this->counterTD(current_pulse, false) * 2.0 * M_PI / motor_hall_ppr * ctrl_frequency_);
  error_ = target_pulse_ - current_pulse;
  error_integ_prev_ = error_integ_;
  error_integ_ += (error_ / ctrl_frequency_);
  double duty = (kp_ * error_ + ki_ * error_integ_ + kd_ * error_dt_);

  if (antiwindup_)
  {
    if (duty > duty_limiter_)
    {
      duty = duty_limiter_;
      if (error_integ_ > error_integ_prev_)
      {
        error_integ_ = error_integ_prev_;
        duty = (kp_ * error_ + ki_ * error_integ_ + kd_ * error_dt_);
      }
    }
    else if (duty < -duty_limiter_)
    {
      duty = -duty_limiter_;
      if (error_integ_ < error_integ_prev_)
      {
        error_integ_ = error_integ_prev_;
        duty = (kp_ * error_ + ki_ * error_integ_ + kd_ * error_dt_);
      }
    }
    if (ki_ * error_integ_ > i_clamp_)
    {
      error_integ_ = i_clamp_ / ki_;
    }
    else if (ki_ * error_integ_ < -i_clamp_)
    {
      error_integ_ = -i_clamp_ / ki_;
    }
  }

  // limit duty value
  duty = std::clamp(duty, -duty_limiter_, duty_limiter_);
  interface_ptr_->setDutyCycle(fabs(target_velocity) < 0.0001 ? 0.0 : duty);
}

void VescWheelController::setControlFrequency(const double frequency)
{
  ctrl_frequency_ = frequency;
}

double VescWheelController::counterTD(const double count_in, bool initialize)
{
  if (initialize)
  {
    counter_changed_single_ = 1;
    for (int i = 0; i < 10; i++)
    {
      counter_changed_log_[i][0] = static_cast<uint16_t>(count_in);
      counter_changed_log_[i][1] = 100;
      counter_td_tmp_[i] = 0;
    }
    return 0.0;
  }

  if (counter_changed_log_[0][0] != static_cast<uint16_t>(count_in))
  {
    for (int i = 0; i < 10; i++)
    {
      counter_changed_log_[10 - i][0] = counter_changed_log_[9 - i][0];
    }
    counter_changed_log_[0][0] = static_cast<uint16_t>(count_in);
    counter_changed_log_[0][1] = counter_changed_single_;
    counter_changed_single_ = 1;
  }
  else
  {
    if (counter_changed_single_ > counter_changed_log_[0][1])
    {
      counter_changed_log_[0][1] = counter_changed_single_;
    }
    if (counter_changed_single_ < 100)
    {
      counter_changed_single_++;
    }
  }

  for (int i = 1; i < 10; i++)
  {
    counter_td_tmp_[10 - i] = counter_td_tmp_[9 - i];
  }
  counter_td_tmp_[0] = static_cast<double>(counter_changed_log_[0][0] - counter_changed_log_[1][0]) /
                       static_cast<double>(counter_changed_log_[0][1]);
  double output = counter_td_tmp_[0];
  if (fabs(output) > 100.0)
  {
    output = 0.0;
  }
  return output;
}
}  // namespace vesc_hw_interface
