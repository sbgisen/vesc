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

#include "vesc_hw_interface/vesc_servo_controller.hpp"
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

namespace vesc_hw_interface
{

VescServoController::VescServoController() : num_rotor_poles_(1), gear_ratio_(1.0), torque_const_(1.0)
{
}

VescServoController::~VescServoController()
{
  interface_ptr_->setDutyCycle(0.0);
}

void VescServoController::init(hardware_interface::HardwareInfo& info,
                               const std::shared_ptr<VescInterface>& interface_ptr, const double gear_ratio,
                               const double torque_const, const int rotor_poles, const int hall_sensors,
                               const int joint_type, const double screw_lead, const double upper_endstop_position,
                               const double lower_endstop_position)
{
  // initializes members
  if (!interface_ptr)
  {
    rclcpp::shutdown();
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
  upper_endstop_position_ = upper_endstop_position;
  lower_endstop_position_ = lower_endstop_position;

  calibration_flag_ = true;
  sensor_initialize_ = true;
  zero_position_ = 0.0;
  error_integ_ = 0.0;
  steps_previous_ = 0;
  position_steps_ = 0;
  calibration_steps_ = 0;
  calibration_previous_position_ = 0.0;
  calibration_rewind_ = false;

  // reads parameters
  kp_ = 50.0;
  if (info.hardware_parameters.find("servo/Kp") != info.hardware_parameters.end())
  {
    kp_ = std::stod(info.hardware_parameters["servo/Kp"]);
  }
  ki_ = 0.0;
  if (info.hardware_parameters.find("servo/Ki") != info.hardware_parameters.end())
  {
    ki_ = std::stod(info.hardware_parameters["servo/Ki"]);
  }
  kd_ = 1.0;
  if (info.hardware_parameters.find("servo/Kd") != info.hardware_parameters.end())
  {
    kd_ = std::stod(info.hardware_parameters["servo/Kd"]);
  }
  i_clamp_ = 1.0;
  if (info.hardware_parameters.find("servo/i_clamp") != info.hardware_parameters.end())
  {
    i_clamp_ = std::stod(info.hardware_parameters["servo/i_clamp"]);
  }
  duty_limiter_ = 1.0;
  if (info.hardware_parameters.find("servo/duty_limiter") != info.hardware_parameters.end())
  {
    duty_limiter_ = std::stod(info.hardware_parameters["servo/duty_limiter"]);
  }
  antiwindup_ = true;
  if (info.hardware_parameters.find("servo/antiwindup") != info.hardware_parameters.end())
  {
    antiwindup_ = info.hardware_parameters["servo/antiwindup"] == "true";
  }
  control_rate_ = 100.0;
  if (info.hardware_parameters.find("servo/control_rate") != info.hardware_parameters.end())
  {
    control_rate_ = std::stod(info.hardware_parameters["servo/control_rate"]);
  }
  calibration_current_ = 0.0;
  if (info.hardware_parameters.find("servo/calibration_current") != info.hardware_parameters.end())
  {
    calibration_current_ = std::stod(info.hardware_parameters["servo/calibration_current"]);
  }
  calibration_strict_current_ = calibration_current_;
  if (info.hardware_parameters.find("servo/calibration_strict_current") != info.hardware_parameters.end())
  {
    calibration_strict_current_ = std::stod(info.hardware_parameters["servo/calibration_strict_current"]);
  }
  calibration_duty_ = 0.1;
  if (info.hardware_parameters.find("servo/calibration_duty") != info.hardware_parameters.end())
  {
    calibration_duty_ = std::stod(info.hardware_parameters["servo/calibration_duty"]);
  }
  calibration_strict_duty_ = calibration_duty_;
  if (info.hardware_parameters.find("servo/calibration_strict_duty") != info.hardware_parameters.end())
  {
    calibration_strict_duty_ = std::stod(info.hardware_parameters["servo/calibration_strict_duty"]);
  }
  calibration_mode_ = CURRENT_;
  if (info.hardware_parameters.find("servo/calibration_mode") != info.hardware_parameters.end())
  {
    calibration_mode_ = info.hardware_parameters["servo/calibration_mode"];
  }
  calibration_position_ = 0.0;
  if (info.hardware_parameters.find("servo/calibration_position") != info.hardware_parameters.end())
  {
    calibration_position_ = std::stod(info.hardware_parameters["servo/calibration_position"]);
  }
  calibration_flag_ = true;
  if (info.hardware_parameters.find("servo/calibration") != info.hardware_parameters.end())
  {
    calibration_flag_ = info.hardware_parameters["servo/calibration"] == "true";
  }
  calibration_result_path_ = "";
  if (info.hardware_parameters.find("servo/calibration_result_path") != info.hardware_parameters.end())
  {
    calibration_result_path_ = info.hardware_parameters["servo/calibration_result_path"];
  }
  if (!calibration_result_path_.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Control] Latest position will be saved to %s",
                calibration_result_path_.data());
  }
  if (!calibration_flag_)
  {
    target_position_previous_ = target_position_;
    sens_position_ = target_position_;

    position_steps_ = sens_position_ * (num_hall_sensors_ * num_rotor_poles_) / gear_ratio_;

    if (joint_type_ == 0 || joint_type_ == 1)
    {
      position_steps_ /= 2.0 * M_PI;
    }
    else if (joint_type_ == 2)
    {
      position_steps_ /= screw_lead_;
    }
    vesc_step_difference_.resetStepDifference(position_steps_);
  }

  bool use_endstop = false;
  if (info.hardware_parameters.find("servo/use_endstop") != info.hardware_parameters.end())
  {
    use_endstop = info.hardware_parameters["servo/use_endstop"] == "true";
  }
  if (use_endstop)
  {
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__node:=" + info.name + "_endstop" });
    node_ = rclcpp::Node::make_shared("_", options);
    endstop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "endstop", rclcpp::SensorDataQoS(),
        std::bind(&VescServoController::endstopCallback, this, std::placeholders::_1));
    while (endstop_sub_->get_publisher_count() == 0)
    {
      RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Control] Waiting for endstop sensor publisher...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  endstop_margin_ = 0.02;
  if (info.hardware_parameters.find("servo/endstop_margin") != info.hardware_parameters.end())
  {
    endstop_margin_ = std::stod(info.hardware_parameters["servo/endstop_margin"]);
  }
  endstop_threshold_ = 0.8;
  if (info.hardware_parameters.find("servo/endstop_threshold") != info.hardware_parameters.end())
  {
    endstop_threshold_ = std::stod(info.hardware_parameters["servo/endstop_threshold"]);
  }
  endstop_window_ = 1;
  if (info.hardware_parameters.find("servo/endstop_window") != info.hardware_parameters.end())
  {
    endstop_window_ = std::stoi(info.hardware_parameters["servo/endstop_window"]);
  }
  endstop_deque_ = std::deque<int>(endstop_window_, 0);
  position_resolution_ = 1.0 / (num_hall_sensors_ * num_rotor_poles_) * gear_ratio_;
  if (joint_type_ == 0 || joint_type_ == 1)
  {
    position_resolution_ = position_resolution_ * 2.0 * M_PI;  // unit: rad
  }
  else if (joint_type_ == 2)
  {
    position_resolution_ = position_resolution_ * screw_lead_;  // unit: m
  }

  // shows parameters
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Gains] P: %f, I: %f, D: %f", kp_, ki_, kd_);
  if (calibration_mode_ == CURRENT_)
  {
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Calibration] Mode: %s, value: %f", CURRENT_.data(),
                calibration_current_);
  }
  else if (calibration_mode_ == DUTY_)
  {
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Calibration] Mode: %s, value: %f", DUTY_.data(),
                calibration_duty_);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "[Servo Calibration] Invalid mode");
  }

  // Smoothing differentiation when hall sensor resolution is insufficient
  bool smooth_diff = true;
  if (info.hardware_parameters.find("servo/enable_smooth_diff") != info.hardware_parameters.end())
  {
    smooth_diff = info.hardware_parameters["servo/enable_smooth_diff"] == "true";
  }
  if (smooth_diff)
  {
    double smooth_diff_max_sampling_time = 1.0;
    if (info.hardware_parameters.find("servo/smooth_diff/max_sample_sec") != info.hardware_parameters.end())
    {
      smooth_diff_max_sampling_time = std::stod(info.hardware_parameters["servo/smooth_diff/max_sample_sec"]);
    }
    int counter_td_vw_max_step = 10;
    if (info.hardware_parameters.find("servo/smooth_diff/max_smooth_step") != info.hardware_parameters.end())
    {
      counter_td_vw_max_step = std::stoi(info.hardware_parameters["servo/smooth_diff/max_smooth_step"]);
    }
    vesc_step_difference_.enableSmooth(control_rate_, smooth_diff_max_sampling_time, counter_td_vw_max_step);
    RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"),
                "[Servo Control] Smooth differentiation enabled, max_sample_sec: %f, max_smooth_step: %d",
                smooth_diff_max_sampling_time, counter_td_vw_max_step);
  }
  // Create timer callback for PID servo control
  // control_timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &VescServoController::controlTimerCallback,
  // this);
  return;
}

void VescServoController::control(const double control_rate)
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

  double error = target_position_ - sens_position_;
  // PID control
  double step_diff = vesc_step_difference_.getStepDifference(position_steps_);
  double current_vel = step_diff * 2.0 * M_PI / (num_rotor_poles_ * num_hall_sensors_) * control_rate * gear_ratio_;
  double target_vel = (target_position_ - target_position_previous_) * control_rate;

  auto average = std::accumulate(endstop_deque_.begin(), endstop_deque_.end(), 0.0) / endstop_deque_.size();
  if (error > 0 && average >= endstop_threshold_)
  {
    RCLCPP_WARN(rclcpp::get_logger("VescHwInterface"), "[Servo Control] Upper endstop signal received. Stop servo.");
    error = 0.0;
    target_vel = 0.0;
    // Wait for target position convergence
    if (std::fabs(target_position_ - target_position_previous_) < std::numeric_limits<double>::epsilon() &&
        std::fabs(target_position_ - upper_endstop_position_) < endstop_margin_)
    {
      zero_position_ = sens_position_ + zero_position_ - upper_endstop_position_;
      sens_position_ = upper_endstop_position_;
      RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Control] Reset position to %f.",
                  upper_endstop_position_);
    }
  }
  else if (error < 0 && average <= -endstop_threshold_)
  {
    RCLCPP_WARN(rclcpp::get_logger("VescHwInterface"), "[Servo Control] Lower endstop signal received. Stop servo.");
    error = 0.0;
    target_vel = 0.0;
    // Wait for target position convergence

    if (std::fabs(target_position_ - target_position_previous_) < std::numeric_limits<double>::epsilon() &&
        std::fabs(target_position_ - lower_endstop_position_) < endstop_margin_)
    {
      zero_position_ = sens_position_ + zero_position_ - lower_endstop_position_;
      sens_position_ = lower_endstop_position_;
      RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[Servo Control] Reset position to %f.",
                  lower_endstop_position_);
    }
  }

  if (std::fabs(error) < position_resolution_)
  {
    error = 0.0;
  }
  double error_dt = target_vel - current_vel;
  error_integ_ += (error / control_rate);
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
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]Gear ratio is set to %f", gear_ratio_);
}

void VescServoController::setTorqueConst(const double torque_const)
{
  torque_const_ = torque_const;
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]Torque constant is set to %f",
              torque_const_);
}

void VescServoController::setRotorPoles(const int rotor_poles)
{
  num_rotor_poles_ = rotor_poles;
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]The number of rotor pole is set to %d",
              num_rotor_poles_);
}

void VescServoController::setHallSensors(const int hall_sensors)
{
  num_hall_sensors_ = hall_sensors;
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]The number of hall sensors is set to %d",
              num_hall_sensors_);
}

void VescServoController::setJointType(const int joint_type)
{
  joint_type_ = joint_type;
}

void VescServoController::setScrewLead(const double screw_lead)
{
  screw_lead_ = screw_lead;
  RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "[VescServoController]Screw lead is set to %f", screw_lead_);
}

double VescServoController::getZeroPosition() const
{
  return zero_position_;
}

void VescServoController::spinSensorData()
{
  if (rclcpp::ok() && node_)
  {
    rclcpp::spin_some(node_);
  }
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
  if (calibration_mode_ == CURRENT_)
  {
    auto sign = calibration_rewind_ ? -1.0 : 1.0;
    interface_ptr_->setCurrent(sign * calibration_current_);
  }
  else if (calibration_mode_ == DUTY_)
  {
    auto sign = calibration_rewind_ ? -1.0 : 1.0;
    interface_ptr_->setDutyCycle(sign * calibration_duty_);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("VescHwInterface"), "Please set the calibration mode surely");
    return false;
  }

  if (calibration_rewind_)
  {
    if (std::fabs(calibration_position_ - sens_position_) > (upper_endstop_position_ - lower_endstop_position_) / 10.0)
    {
      calibration_current_ = calibration_strict_current_;
      calibration_duty_ = calibration_strict_duty_;
      calibration_rewind_ = false;
    }
    return false;
  }

  if (std::accumulate(endstop_deque_.begin(), endstop_deque_.end(), 0.0) != 0.0)
  {
    zero_position_ = sens_position_ + zero_position_ - calibration_position_;
    if ((calibration_mode_ == CURRENT_ &&
         std::fabs(calibration_current_ - calibration_strict_current_) < std::numeric_limits<double>::epsilon()) ||
        (calibration_mode_ == DUTY_ &&
         std::fabs(calibration_duty_ - calibration_strict_duty_) < std::numeric_limits<double>::epsilon()))
    {
      target_position_ = calibration_position_;
      RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Calibration Finished");
      calibration_flag_ = false;
      return true;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Calibrate with strict current/duty.");
      calibration_rewind_ = true;
      return false;
    }
  }

  calibration_steps_++;

  if (calibration_steps_ % 20 == 0)
  {
    if (std::abs(sens_position_ - calibration_previous_position_) <= std::numeric_limits<double>::epsilon())
    {
      // finishes calibrating
      calibration_steps_ = 0;
      zero_position_ = sens_position_ - calibration_position_;
      target_position_ = calibration_position_;
      RCLCPP_INFO(rclcpp::get_logger("VescHwInterface"), "Calibration Finished");
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

// void VescServoController::controlTimerCallback(const ros::TimerEvent& e)
// {
//   control();
//   interface_ptr_->requestState();
// }

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

    if (joint_type_ == 0 || joint_type_ == 1)
    {
      sens_position_ = sens_position_ * 2.0 * M_PI;         // unit: rad
      sens_velocity_ = sens_velocity_ / 60.0 * 2.0 * M_PI;  // unit: rad/s
    }
    else if (joint_type_ == 2)
    {
      sens_position_ = sens_position_ * screw_lead_;         // unit: m
      sens_velocity_ = sens_velocity_ / 60.0 * screw_lead_;  // unit: m/s
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

void VescServoController::endstopCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
  endstop_deque_.pop_front();
  if (!msg->data)
  {
    endstop_deque_.push_back(0);
  }
  else
  {
    if (calibration_flag_)
    {
      if (calibration_mode_ == CURRENT_)
      {
        if (std::signbit(calibration_current_))
        {
          endstop_deque_.push_back(-1);
        }
        else
        {
          endstop_deque_.push_back(1);
        }
      }
      else if (calibration_mode_ == DUTY_)
      {
        if (std::signbit(calibration_duty_))
        {
          endstop_deque_.push_back(-1);
        }
        else
        {
          endstop_deque_.push_back(1);
        }
      }
    }
    else if (std::fabs(sens_position_ - upper_endstop_position_) < std::fabs(sens_position_ - lower_endstop_position_))
    {
      endstop_deque_.push_back(1);
    }
    else
    {
      endstop_deque_.push_back(-1);
    }
  }
}

}  // namespace vesc_hw_interface
