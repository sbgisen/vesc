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

  ROS_INFO("[Motor Gains] P: %f, I: %f, D: %f", kp_, ki_, kd_);
  ROS_INFO("[Motor Gains] I clamp: %f, Antiwindup: %s", i_clamp_, antiwindup_ ? "true" : "false");

  reset_ = true;
}

void VescWheelController::control(const double target_velocity, const double current_steps)
{
  if (reset_)
  {
    target_steps_ = current_steps;
    error_ = 0.0;
    error_dt_ = 0.0;
    error_integ_ = 0.0;
    error_integ_prev_ = 0.0;
    this->counterTD(current_steps, true);
  }
  else
  {  // convert rad/s to steps
    target_steps_ +=
        target_velocity * (num_rotor_poles_ * num_hall_sensors_) / (2 * M_PI) / control_rate_ / gear_ratio_;
  }

  // overflow check
  if ((target_steps_ - current_steps) > std::numeric_limits<int>::max())
  {
    target_steps_ +=
        static_cast<double>(std::numeric_limits<int>::min()) - static_cast<double>(std::numeric_limits<int>::max());
  }
  else if ((target_steps_ - current_steps) < std::numeric_limits<int>::min())
  {
    target_steps_ +=
        static_cast<double>(std::numeric_limits<int>::max()) - static_cast<double>(std::numeric_limits<int>::min());
  }

  // pid control
  double current_vel = this->counterTD(current_steps, false) * 2.0 * M_PI / (num_rotor_poles_ * num_hall_sensors_) *
                       control_rate_ * gear_ratio_;
  error_dt_ = target_velocity - current_vel;
  error_ = (target_steps_ - current_steps) * 2.0 * M_PI / (num_rotor_poles_ * num_hall_sensors_);
  error_integ_prev_ = error_integ_;
  error_integ_ += (error_ / control_rate_);
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
  interface_ptr_->setDutyCycle(std::abs(target_velocity) < 0.0001 ? 0.0 : duty);

  // disable pid when velocity is 0
  reset_ = std::abs(target_velocity) < 0.0001;
}

double VescWheelController::counterTD(const double count_in, bool reset)
{
  if (reset)
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

void VescWheelController::setGearRatio(const double gear_ratio)
{
  gear_ratio_ = gear_ratio;
}

void VescWheelController::setRotorPoles(const int rotor_poles)
{
  num_rotor_poles_ = static_cast<double>(rotor_poles);
  num_rotor_pole_pairs_ = num_rotor_poles_ / 2;
}

void VescWheelController::setHallSensors(const int hall_sensors)
{
  num_hall_sensors_ = static_cast<double>(hall_sensors);
}

void VescWheelController::setControlRate(const double control_rate)
{
  control_rate_ = control_rate;
}

}  // namespace vesc_hw_interface
