/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#include "vesc_driver/vesc_packet.hpp"
#include <cstring>

namespace vesc_driver
{

VescFrame::VescFrame(const BufferRangeConst& payload) 
{
  payload_.resize(std::distance(boost::begin(payload), boost::end(payload)));
  payload_.assign(boost::begin(payload), boost::end(payload));
}

VescFrame::VescFrame(const int16_t payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);
  payload_.resize(payload_size);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 * @param name Data name
 * @param payload_size Specified payload size
 * @param payload_id ID of payload
 **/
VescPacket::VescPacket(const std::string& name, const int16_t payload_size,
                   const COMM_PACKET_ID cmd)
    : VescFrame(payload_size), name_(name) {
  int16_t packet_id = static_cast<int16_t>(cmd);
  assert(packet_id >= 0 && packet_id < 256);
  // assert(boost::distance(payload_end_) > 0);
  assert(static_cast<int16_t>(payload_.size()) == payload_size);
  setPayloadId(packet_id);
}

/**
 * @brief Constructor
 * @param name Data name
 * @param raw Pointer of a frame
 **/
VescPacket::VescPacket(const std::string& name, std::shared_ptr<VescFrame> raw)
    : VescFrame(*raw), name_(name) {
  // not sure what this is for
  // uint16_t original_payload_size = std::distance(payload_end_.first,
  // payload_end_.second); payload_end_.first = frame_.begin() + 2;
  // payload_end_.second = std::min(payload_end_.first + original_payload_size,
  // frame_.end());
}

/**
 * @brief Constructor
 * @param name Data name
 * @param payload_size Specified payload size
 * @param payload_id ID of payload
 **/
VescCanPacket::VescCanPacket(const std::string& name, const int16_t payload_size, const CAN_PACKET_ID cmd)
    : VescFrame(payload_size), name_(name), can_packet_id_(cmd) {
  int16_t packet_id = static_cast<int16_t>(cmd);
  assert(packet_id >= 0 && packet_id < 256);
  assert(static_cast<int16_t>(payload_.size()) == payload_size);

}

/**
 * @brief Constructor
 * @param name Data name
 * @param raw Pointer of a frame
 **/
VescCanPacket::VescCanPacket(const std::string& name, std::shared_ptr<VescFrame> raw)
    : VescFrame(*raw), name_(name) {
}

/**
 * @brief Reads a value from the buffer
 * @param map_id start address to read
 * @param size the number of bytes to read
 * @return Required value
 **/
double VescPacket::readBuffer(const uint8_t map_id, const uint8_t size) const
{
  int32_t value = 0;
  switch (size)
  {
    case 2:
      value += static_cast<int32_t>(*(payload_.begin() + map_id) << 8);
      value += static_cast<int32_t>(*(payload_.begin()  + map_id + 1));
      break;
    case 4:
      value += static_cast<int32_t>(*(payload_.begin()  + map_id) << 24);
      value += static_cast<int32_t>(*(payload_.begin()  + map_id + 1) << 16);
      value += static_cast<int32_t>(*(payload_.begin()  + map_id + 2) << 8);
      value += static_cast<int32_t>(*(payload_.begin()  + map_id + 3));
      break;
  }

  return static_cast<double>(value);
}

/**
 * @brief Reads a value from the buffer that is serialized with buffer_append_float32_auto function
 * @param map_id start address to read
 * @param size the number of bytes to read
 * @return Required value
 **/
double VescPacket::readAutoBuffer(const int map_id) const
{
  uint32_t bits = 0;
  bits |= static_cast<uint32_t>(*(payload_.begin()  + map_id)) << 24;
  bits |= static_cast<uint32_t>(*(payload_.begin()  + map_id + 1)) << 16;
  bits |= static_cast<uint32_t>(*(payload_.begin()  + map_id + 2)) << 8;
  bits |= static_cast<uint32_t>(*(payload_.begin()  + map_id + 3));

  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return static_cast<double>(value);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 * @param raw Pointer of VescFrame
 **/
VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescFrame> raw) : VescPacket("FWVersion", raw)
{
}

/**
 * @brief Gets major farmware version
 * @return Major farmware version
 **/
int16_t VescPacketFWVersion::fwMajor() const
{
  return *(payload_.begin()+1);
}

/**
 * @brief Gets minor farmware version
 * @return Minor farmware version
 **/
int16_t VescPacketFWVersion::fwMinor() const
{
  return *(payload_.begin() + 2);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestFWVersion::VescPacketRequestFWVersion() : VescPacket("RequestFWVersion", 1, COMM_PACKET_ID::COMM_FW_VERSION)
{
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketValues::VescPacketValues(std::shared_ptr<VescFrame> raw) : VescPacket("Values", raw)
{
}

/**
 * @brief Gets temperature of MOSFETs
 * @return Temperature of MOSFETs
 **/
double VescPacketValues::getMosTemp() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::TEMP_MOS), 2) / 10.0;
}

/**
 * @brief Gets temperature of the motor
 * @return Temperature of the motor
 **/
double VescPacketValues::getMotorTemp() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::TEMP_MOTOR), 2) / 10.0;
}

/**
 * @brief Gets motor current
 * @return Motor current
 **/
double VescPacketValues::getMotorCurrent() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::CURRENT_MOTOR), 4) / 100.0;
}

/**
 * @brief Gets input current
 * @return Input current
 **/
double VescPacketValues::getInputCurrent() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::CURRENT_IN), 4) / 100.0;
}

/**
 * @brief Gets the current duty value
 * @return The current duty value
 **/
double VescPacketValues::getDuty() const
{
  int16_t duty_raw = static_cast<int32_t>(readBuffer(static_cast<uint8_t>(PACKET_MAP::DUTY_NOW), 2));

  // inverts to derive a negative value
  if (duty_raw > 1000)
  {
    duty_raw = !duty_raw;
  }

  return static_cast<double>(duty_raw) / 1000.0;
}

/**
 * @brief Gets the current angular velocity
 * @return The current angular velocity
 **/
double VescPacketValues::getVelocityERPM() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::ERPM), 4);
}

/**
 * @brief Gets input voltage
 * @return Input voltage
 **/
double VescPacketValues::getInputVoltage() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::VOLTAGE_IN), 2) / 10.0;
}

/**
 * @brief Gets consumed charge
 * @return Consumed charge
 **/
double VescPacketValues::getConsumedCharge() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::AMP_HOURS), 4) / 10000.0;
}

/**
 * @brief Gets input charge
 * @return Input charge
 **/
double VescPacketValues::getInputCharge() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::AMP_HOURS_CHARGED), 4) / 10000.0;
}

/**
 * @brief Gets consumed power
 * @return Consumed power
 **/
double VescPacketValues::getConsumedPower() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::TACHOMETER), 4) / 10000.0;
}

/**
 * @brief Gets input power
 * @return Input power
 **/
double VescPacketValues::getInputPower() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::WATT_HOURS_CHARGED), 4) / 10000.0;
}

/**
 * @brief Gets the current tachometer value
 * @return The current tachometer value
 **/
double VescPacketValues::getTachometer() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::TACHOMETER), 4);
}

/**
 * @brief Gets absolute displacement in tachometer
 * @return Absolute displacement in tachometer
 **/
double VescPacketValues::getDisplacement() const
{
  return readBuffer(static_cast<uint8_t>(PACKET_MAP::FAULT_CODE), 4);
}

/**
 * @brief Gets fault code
 * @return Fault code
 **/
int VescPacketValues::getFaultCode() const
{
  return static_cast<int32_t>(*(payload_.begin() + static_cast<uint8_t>(PACKET_MAP::FAULT_CODE)));
}

/**
 * @brief Gets the position in deg.
 * @return The current position between 0 to 360 deg.
 **/
 double VescPacketValues::getPosition() const
 {
   return readBuffer(static_cast<uint8_t>(PACKET_MAP::PID_POS), 4) / 1000000.0;
 }
// double VescPacketValues::readBuffer(const PACKET_MAP  packet_map, const uint8_t size) const
// {
//   uint8_t map_id = static_cast<uint8_t>(packet_map);
//   int32_t value = 0;
//   switch (size)
//   {
//     case 2:
//       value += static_cast<int32_t>(*(payload_.begin() + map_id) << 8);
//       value += static_cast<int32_t>(*(payload_.begin()  + map_id + 1));
//       break;
//     case 4:
//       value += static_cast<int32_t>(*(payload_.begin()  + map_id) << 24);
//       value += static_cast<int32_t>(*(payload_.begin()  + map_id + 1) << 16);
//       value += static_cast<int32_t>(*(payload_.begin()  + map_id + 2) << 8);
//       value += static_cast<int32_t>(*(payload_.begin()  + map_id + 3));
//       break;
//   }
//   return value;
// }

 /**
 * @brief Gets controller id
 * @return Fault code
 **/
int VescPacketValues::getControllerID() const
{
  return static_cast<int32_t>(*(payload_.begin() + static_cast<uint8_t>(PACKET_MAP::CONTROLLER_ID)));
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestValues::VescPacketRequestValues() : VescPacket("RequestValues", 1, COMM_PACKET_ID::COMM_GET_VALUES)
{
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketMCConf::VescPacketMCConf(std::shared_ptr<VescFrame> raw) : VescPacket("MCConfiguration", raw)
{
  int map_id = 2;
  config_.signature = static_cast<uint32_t>(readAutoBuffer(map_id)); map_id += 4;
  config_.pwm_mode = static_cast<PWM_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.comm_mode = static_cast<COMM_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.motor_type = static_cast<MOTOR_TYPE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.sensor_mode = static_cast<SENSOR_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.l_current_max = readAutoBuffer(map_id); map_id += 4;
  config_.l_current_min = readAutoBuffer(map_id); map_id += 4;
  config_.l_in_current_max = readAutoBuffer(map_id); map_id += 4;
  config_.l_in_current_min = readAutoBuffer(map_id); map_id += 4;
  config_.l_abs_current_max = readAutoBuffer(map_id); map_id += 4;
  config_.l_min_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.l_max_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.l_erpm_start = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_max_erpm_fbrake = readAutoBuffer(map_id); map_id += 4;
  config_.l_max_erpm_fbrake_cc = readAutoBuffer(map_id); map_id += 4;
  config_.l_min_vin = readAutoBuffer(map_id); map_id += 4;
  config_.l_max_vin = readAutoBuffer(map_id); map_id += 4;
  config_.l_battery_cut_start = readAutoBuffer(map_id); map_id += 4;
  config_.l_battery_cut_end = readAutoBuffer(map_id); map_id += 4;
  config_.l_slow_abs_current = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.l_temp_fet_start = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_fet_end = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_motor_start = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_motor_end = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.l_temp_accel_dec = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_min_duty = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_max_duty = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_watt_max = readAutoBuffer(map_id); map_id += 4;
  config_.l_watt_min = readAutoBuffer(map_id); map_id += 4;
  config_.l_current_max_scale = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_current_min_scale = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.l_duty_start = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.sl_min_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.sl_min_erpm_cycle_int_limit = readAutoBuffer(map_id); map_id += 4;
  config_.sl_max_fullbreak_current_dir_change = readAutoBuffer(map_id); map_id += 4;
  config_.sl_cycle_int_limit = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.sl_phase_advance_at_br = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.sl_cycle_int_rpm_br = readAutoBuffer(map_id); map_id += 4;
  config_.sl_bemf_coupling_k = readAutoBuffer(map_id); map_id += 4;
  config_.hall_table[0] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_table[1] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_table[2] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_table[3] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_table[4] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_table[5] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_table[6] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_table[7] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.hall_sl_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.foc_current_kp = readAutoBuffer(map_id); map_id += 4;
  config_.foc_current_ki = readAutoBuffer(map_id); map_id += 4;
  config_.foc_f_zv = readAutoBuffer(map_id); map_id += 4;
  config_.foc_dt_us = readAutoBuffer(map_id); map_id += 4;
  config_.foc_encoder_inverted = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_encoder_offset = readAutoBuffer(map_id); map_id += 4;
  config_.foc_encoder_ratio = readAutoBuffer(map_id); map_id += 4;
  config_.foc_encoder_sin_gain = readAutoBuffer(map_id); map_id += 4;
  config_.foc_encoder_cos_gain = readAutoBuffer(map_id); map_id += 4;
  config_.foc_encoder_sin_offset = readAutoBuffer(map_id); map_id += 4;
  config_.foc_encoder_cos_offset = readAutoBuffer(map_id); map_id += 4;
  config_.foc_encoder_sincos_filter_constant = readAutoBuffer(map_id); map_id += 4;
  config_.foc_sensor_mode = static_cast<FOC_SENSOR_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_pll_kp = readAutoBuffer(map_id); map_id += 4;
  config_.foc_pll_ki = readAutoBuffer(map_id); map_id += 4;
  config_.foc_motor_l = readAutoBuffer(map_id); map_id += 4;
  config_.foc_motor_ld_lq_diff = readAutoBuffer(map_id); map_id += 4;
  config_.foc_motor_r = readAutoBuffer(map_id); map_id += 4;
  config_.foc_motor_flux_linkage = readAutoBuffer(map_id); map_id += 4;
  config_.foc_observer_gain = readAutoBuffer(map_id); map_id += 4;
  config_.foc_observer_gain_slow = readAutoBuffer(map_id); map_id += 4;
  config_.foc_observer_offset = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_duty_dowmramp_kp = readAutoBuffer(map_id); map_id += 4;
  config_.foc_duty_dowmramp_ki = readAutoBuffer(map_id); map_id += 4;
  config_.foc_openloop_rpm = readAutoBuffer(map_id); map_id += 4;
  config_.foc_openloop_rpm_low = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_d_gain_scale_start = readAutoBuffer(map_id); map_id += 4;
  config_.foc_d_gain_scale_max_mod = readAutoBuffer(map_id); map_id += 4;
  config_.foc_sl_openloop_hyst = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_sl_openloop_time_lock = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_sl_openloop_time_ramp = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_sl_openloop_time = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_hall_table[0] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_table[1] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_table[2] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_table[3] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_table[4] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_table[5] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_table[6] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_table[7] = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hall_interp_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.foc_sl_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.foc_sample_v0_v7 = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_sample_high_current = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_sat_comp = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_temp_comp = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_temp_comp_base_temp = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.foc_current_filter_const = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_cc_decoupling = static_cast<FOC_CC_DECOUPLING_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_observer_type = static_cast<FOC_OBSERVER_TYPE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_hfi_voltage_start = readAutoBuffer(map_id); map_id += 4;
  config_.foc_hfi_voltage_run = readAutoBuffer(map_id); map_id += 4;
  config_.foc_hfi_voltage_max = readAutoBuffer(map_id); map_id += 4;
  config_.foc_sl_erpm_hfi = readAutoBuffer(map_id); map_id += 4;
  config_.foc_hfi_start_samples = readBuffer(map_id, 2); map_id += 2;
  config_.foc_hfi_obs_ovr_sec = readAutoBuffer(map_id); map_id += 4;
  config_.foc_hfi_samples = static_cast<FOC_HFI_SAMPLES>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_offsets_cal_on_boot = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_offsets_current[0] = readAutoBuffer(map_id); map_id += 4;
  config_.foc_offsets_current[1] = readAutoBuffer(map_id); map_id += 4;
  config_.foc_offsets_current[2] = readAutoBuffer(map_id); map_id += 4;
  config_.foc_offsets_voltage[0] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage[1] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage[2] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage_undriven[0] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage_undriven[1] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_offsets_voltage_undriven[2] = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_phase_filter_enable = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_phase_filter_max_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.foc_mtpa_mode = static_cast<MTPA_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.foc_fw_current_max = readAutoBuffer(map_id); map_id += 4;
  config_.foc_fw_duty_start = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.foc_fw_ramp_time  = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.foc_fw_q_current_factor = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.gpd_buffer_notify_left = readBuffer(map_id, 2); map_id += 2;
  config_.gpd_buffer_interpol    = readBuffer(map_id, 2); map_id += 2;
  config_.gpd_current_filter_const = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.gpd_current_kp = readAutoBuffer(map_id); map_id += 4;
  config_.gpd_current_ki = readAutoBuffer(map_id); map_id += 4;
  config_.sp_pid_loop_rate = static_cast<PID_RATE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.s_pid_kp = readAutoBuffer(map_id); map_id += 4;
  config_.s_pid_ki = readAutoBuffer(map_id); map_id += 4;
  config_.s_pid_kd = readAutoBuffer(map_id); map_id += 4;
  config_.s_pid_kd_filter = readBuffer(map_id, 2) / 10000.0; map_id += 2;
  config_.s_pid_min_erpm = readAutoBuffer(map_id); map_id += 4;
  config_.s_pid_allow_braking = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.s_pid_ramp_erpms_s = readAutoBuffer(map_id); map_id += 4;
  config_.p_pid_kp = readAutoBuffer(map_id); map_id += 4;
  config_.p_pid_ki = readAutoBuffer(map_id); map_id += 4;
  config_.p_pid_kd = readAutoBuffer(map_id); map_id += 4;
  config_.p_pid_kd_proc = readAutoBuffer(map_id); map_id += 4;
  config_.p_pid_kd_filter = readAutoBuffer(map_id); map_id += 4;
  config_.p_pid_ang_div = readAutoBuffer(map_id); map_id += 4;
  config_.p_pid_gain_dec_angle = readBuffer(map_id, 2) / 10.0; map_id += 2;
  config_.p_pid_offset = readAutoBuffer(map_id); map_id += 4;
  config_.cc_startup_boost_duty = readAutoBuffer(map_id); map_id += 4;
  config_.cc_min_current = readAutoBuffer(map_id); map_id += 4;
  config_.cc_gain = readAutoBuffer(map_id); map_id += 4;
  config_.cc_ramp_step_max = readAutoBuffer(map_id); map_id += 4;
  config_.m_fault_stop_time_ms = readBuffer(map_id, 4); map_id += 4;
  config_.m_duty_ramp_step = readAutoBuffer(map_id); map_id += 4;
  config_.m_current_backoff_gain = readAutoBuffer(map_id); map_id += 4;
  config_.m_encoder_counts = readAutoBuffer(map_id); map_id += 4;
  config_.m_sensor_port_mode = static_cast<SENSOR_PORT_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.m_invert_direction = static_cast<bool>(*(payload_.begin() + map_id)); map_id += 1;
  config_.m_drv8301_oc_mode = static_cast<DRV8301_OC_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.m_drv8301_oc_adj = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.m_bldc_f_sw_min = readAutoBuffer(map_id); map_id += 4;
  config_.m_bldc_f_sw_max = readAutoBuffer(map_id); map_id += 4;
  config_.m_dc_f_sw = readAutoBuffer(map_id); map_id += 4;
  config_.m_ntc_motor_beta = readAutoBuffer(map_id); map_id += 4;
  config_.m_out_aux_mode = static_cast<OUT_AUX_MODE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.m_motor_temp_sens_type = static_cast<TEMP_SENSOR_TYPE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.m_ptc_motor_coeff = readAutoBuffer(map_id); map_id += 4;
  config_.m_hall_extra_samples = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.si_motor_poles = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.si_gear_ratio = readAutoBuffer(map_id); map_id += 4;
  config_.si_wheel_diameter = readAutoBuffer(map_id); map_id += 4;
  config_.si_battery_type = static_cast<BATTERY_TYPE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.si_battery_cells = static_cast<int>(*(payload_.begin() + map_id)); map_id += 1;
  config_.si_battery_ah = readAutoBuffer(map_id); map_id += 4;
  config_.si_motor_nl_current = readAutoBuffer(map_id); map_id += 4;
  config_.bms.type = static_cast<BMS_TYPE>(*(payload_.begin() + map_id)); map_id += 1;
  config_.bms.t_limit_start = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.bms.t_limit_end   = readBuffer(map_id, 2) / 100.0; map_id += 2;
  config_.bms.soc_limit_start = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.bms.soc_limit_end   = readBuffer(map_id, 2) / 1000.0; map_id += 2;
  config_.bms.fwd_can_mode = static_cast<BMS_FWD_CAN_MODE>(*(payload_.begin() + map_id)); map_id += 1;
}

/**
 * @brief Get MC configuration
 * @return The MC configuration
 **/
MCConfiguration VescPacketMCConf::getConfig() const
{
  return config_;
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestMCConf::VescPacketRequestMCConf() : VescPacket("RequestMCConf", 1, COMM_PACKET_ID::COMM_GET_MCCONF)
{
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetDuty::VescPacketSetDuty(double duty) : VescPacket("SetDuty", 5, COMM_PACKET_ID::COMM_SET_DUTY)
{
  // checks the range of duty
  if (duty > 1.0)
  {
    duty = 1.0;
  }
  else if (duty < -1.0)
  {
    duty = -1.0;
  }

  const int32_t v = static_cast<int32_t>(duty * 100000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),3);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),4);

}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetCurrent::VescPacketSetCurrent(double current) : VescPacket("SetCurrent", 5, COMM_PACKET_ID::COMM_SET_CURRENT)
{
  const int32_t v = static_cast<int32_t>(current * 1000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),3);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),4);


}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake)
  : VescPacket("SetCurrentBrake", 5, COMM_PACKET_ID::COMM_SET_CURRENT_BRAKE)
{
  const int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),3);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),4);

}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetVelocityERPM::VescPacketSetVelocityERPM(int32_t vel_erpm) : VescPacket("SetRPM", 5, COMM_PACKET_ID::COMM_SET_ERPM)
{
  const int32_t v = vel_erpm;

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),3);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),4);


}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetPos::VescPacketSetPos(double pos) : VescPacket("SetPos", 5, COMM_PACKET_ID::COMM_SET_POS)
{
  /** @todo range check pos */

  const int32_t v = static_cast<int32_t>(pos * 1000000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),0);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),3);


}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos) : VescPacket("SetServoPos", 3, COMM_PACKET_ID::COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  uint16_t v = static_cast<uint16_t>(servo_pos * 1000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),0);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),1);

}

/*------------------------------------------------------------------*/


/**
 * @brief Constructor
 **/
VescCanPacketSetDuty::VescCanPacketSetDuty(double duty) : VescCanPacket("SetDuty", 5, CAN_PACKET_ID::CAN_PACKET_SET_DUTY)
{
  // checks the range of duty
  if (duty > 1.0)
  {
    duty = 1.0;
  }
  else if (duty < -1.0)
  {
    duty = -1.0;
  }

  const int32_t v = static_cast<int32_t>(duty * 100000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),0);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),3);

}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescCanPacketSetCurrent::VescCanPacketSetCurrent(double current) : VescCanPacket("SetCurrent", 5, CAN_PACKET_ID::CAN_PACKET_SET_CURRENT)
{
  const int32_t v = static_cast<int32_t>(current * 1000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),0);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),3);


}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescCanPacketSetCurrentBrake::VescCanPacketSetCurrentBrake(double current_brake)
  : VescCanPacket("SetCurrentBrake", 5, CAN_PACKET_ID::CAN_PACKET_SET_CURRENT_BRAKE)
{
  const int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),0);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),3);

}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescCanPacketSetVelocityERPM::VescCanPacketSetVelocityERPM(double vel_erpm) : VescCanPacket("SetERPM", 5, CAN_PACKET_ID::CAN_PACKET_SET_RPM)
{
  const int32_t v = static_cast<int32_t>(vel_erpm);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),0);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),3);


}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescCanPacketSetPos::VescCanPacketSetPos(double pos) : VescCanPacket("SetPos", 5, CAN_PACKET_ID::CAN_PACKET_SET_POS)
{
  /** @todo range check pos */
  const int32_t v = static_cast<int32_t>(pos * 1000000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),0);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),3);


}

/*------------------------------------------------------------------*/

}  // namespace vesc_driver
