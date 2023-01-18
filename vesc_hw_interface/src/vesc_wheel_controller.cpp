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
  nh.param("motor/control_rate", control_rate_, 100.0);

  ROS_INFO("[Motor Gains] P: %f, I: %f, D: %f", kp_, ki_, kd_);
  ROS_INFO("[Motor Gains] I clamp: %f, Antiwindup: %s", i_clamp_, antiwindup_ ? "true" : "false");
  ROS_INFO("[Motor Control] control_rate: %f", control_rate_);

  // Params for counterTDVariableWindow
  nh.param<bool>("motor/enable_smooth_diff", smooth_diff_, true);
  double smooth_diff_max_sampling_time;
  nh.param<double>("motor/smooth_diff/max_sample_sec", smooth_diff_max_sampling_time, 1.0);
  nh.param<int>("motor/smooth_diff/max_smooth_step", counter_td_vw_max_step_, 10);
  counter_td_vw_max_window_size_ = std::max(1, static_cast<int>(control_rate_ * smooth_diff_max_sampling_time));
  counter_td_vw_window_.resize(counter_td_vw_max_window_size_ + 1);
  if (smooth_diff_)
  {
    ROS_INFO(
        "[Motor Control] Smooth differentiation enabled, max_sample_sec: %f, max_smooth_step: %d, "
        "max_window_size:%d,",
        smooth_diff_max_sampling_time, counter_td_vw_max_step_, counter_td_vw_max_window_size_);
  }

  reset_ = true;
  initialize_ = true;
  position_sens_ = 0.0;
  velocity_reference_ = 0.0;
  position_steps_ = 0.0;
  prev_steps_ = 0.0;
  velocity_sens_ = 0.0;
  effort_sens_ = 0.0;

  control_timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &VescWheelController::controlTimerCallback, this);
}

void VescWheelController::control(const double target_velocity, const double current_steps, bool reset)
{
  if (reset)
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
  double step_diff = this->counterTD(current_steps, false);
  double current_vel = step_diff * 2.0 * M_PI / (num_rotor_poles_ * num_hall_sensors_) * control_rate_ * gear_ratio_;
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
  interface_ptr_->setDutyCycle(fabs(target_velocity) < 0.0001 ? 0.0 : duty);
}

double VescWheelController::counterTD(const double count_in, bool reset)
{
  double output;
  if (smooth_diff_)
  {
    output = this->counterTDVariableWindow(count_in, reset);
  }
  else
  {
    output = this->counterTDRaw(count_in, reset);
  }
  return output;
}

double VescWheelController::counterTDRaw(const double count_in, bool reset)
{
  if (reset)
  {
    counter_td_raw_prev_ = static_cast<int16_t>(count_in);
    return 0.0;
  }
  int16_t step_diff = static_cast<int16_t>(count_in) - counter_td_raw_prev_;
  double output = static_cast<double>(step_diff);
  counter_td_raw_prev_ = static_cast<int16_t>(count_in);
  return output;
}

double VescWheelController::counterTDVariableWindow(const double count_in, bool reset)
{
  if (reset)
  {
    for (int i = 0; i < counter_td_vw_window_.size(); i++)
    {
      counter_td_vw_window_[i] = static_cast<int16_t>(count_in);
    }
    return 0.0;
  }
  // Increment buffer
  for (int i = counter_td_vw_window_.size() - 1; i > 0; i--)
  {
    counter_td_vw_window_[i] = counter_td_vw_window_[i - 1];
  }
  counter_td_vw_window_[0] = static_cast<int16_t>(count_in);
  // Calculate window size
  int latest_step_diff = abs(counter_td_vw_window_[0] - counter_td_vw_window_[1]);
  int window_size = counter_td_vw_max_window_size_;
  if (latest_step_diff > counter_td_vw_max_step_ - 1)
  {
    window_size = 1;
  }
  else if (latest_step_diff > 0)
  {
    window_size = counter_td_vw_max_window_size_ *
                  (1.0 - static_cast<double>(latest_step_diff) / static_cast<double>(counter_td_vw_max_step_));
  }
  // Get output
  int16_t step_diff = counter_td_vw_window_[0] - counter_td_vw_window_[window_size];
  double output = static_cast<double>(step_diff) / static_cast<double>(window_size);
  // ROS_WARN("[VescWheelController]window,step0,step1,latest_step_diff,output: %d, %d, %d, %d, %f", window_size,
  //          counter_td_vw_window_[0], counter_td_vw_window_[window_size], latest_step_diff, output);
  return output;
}

void VescWheelController::setTargetVelocity(const double velocity_reference)
{
  velocity_reference_ = velocity_reference;
}

void VescWheelController::setGearRatio(const double gear_ratio)
{
  gear_ratio_ = gear_ratio;
  ROS_INFO("[VescWheelController]Gear ratio is set to %f", gear_ratio_);
}

void VescWheelController::setTorqueConst(const double torque_const)
{
  torque_const_ = torque_const;
  ROS_INFO("[VescWheelController]Torque constant is set to %f", torque_const_);
}

void VescWheelController::setRotorPoles(const int rotor_poles)
{
  num_rotor_poles_ = static_cast<double>(rotor_poles);
  num_rotor_pole_pairs_ = num_rotor_poles_ / 2;
  ROS_INFO("[VescWheelController]The number of rotor poles is set to %d", rotor_poles);
}

void VescWheelController::setHallSensors(const int hall_sensors)
{
  num_hall_sensors_ = static_cast<double>(hall_sensors);
  ROS_INFO("[VescWheelController]The number of hall sensors is set to %d", hall_sensors);
}

double VescWheelController::getPositionSens()
{
  return position_sens_;
}

double VescWheelController::getVelocitySens()
{
  return velocity_sens_;
}

double VescWheelController::getEffortSens()
{
  return effort_sens_;
}

void VescWheelController::controlTimerCallback(const ros::TimerEvent& e)
{
  control(velocity_reference_, position_steps_, reset_);
  interface_ptr_->requestState();
  reset_ = fabs(velocity_reference_) < 0.0001;  // disable PID control when command is 0
}

void VescWheelController::updateSensor(const std::shared_ptr<const VescPacket>& packet)
{
  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);
    const double current = values->getMotorCurrent();
    const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_rotor_pole_pairs_) * gear_ratio_;
    const int steps = static_cast<int>(values->getPosition());
    if (initialize_)
    {
      prev_steps_ = steps;
      initialize_ = false;
    }
    position_steps_ += static_cast<double>(steps - prev_steps_);
    prev_steps_ = steps;

    position_sens_ = (position_steps_ * 2.0 * M_PI) /
                      (num_rotor_poles_ * num_hall_sensors_) * gear_ratio_;  // convert steps to rad
    velocity_sens_ = velocity_rpm * 2 * M_PI / 60;                           // convert rpm to rad/s
    effort_sens_ = current * torque_const_ / gear_ratio_;                    // unit: Nm or N
  }
}
}  // namespace vesc_hw_interface
