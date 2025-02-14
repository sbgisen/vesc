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

namespace vesc_driver
{

VescPayload::VescPayload(const BufferRangeConst& payload) 
{
  payload_.resize(std::distance(boost::begin(payload), boost::end(payload)));
  payload_.assign(boost::begin(payload), boost::end(payload));
}

VescPayload::VescPayload(const int16_t payload_size)
{
  assert(payload_size >= 0 && payload_size <= VESC_MAX_PAYLOAD_SIZE);
  payload_.resize(payload_size);
}

/**
 * @brief Constructor
 * @param payload_size Specified payload size
 **/
VescFrame::VescFrame(const int16_t payload_size):payload_(payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256)
  {
    // single byte payload size
    frame_header_.resize(VESC_MIN_HEADER_SIZE);
    *(frame_header_.begin()) = 2;
    *(frame_header_.begin() + 1) = payload_size;
    
  }
  else
  {
    // two byte payload size
    frame_header_.resize(VESC_MIN_HEADER_SIZE + 1);
    *(frame_header_.begin()) = 3;
    *(frame_header_.begin() + 1) = payload_size >> 8;
    *(frame_header_.begin() + 2) = payload_size & 0xFF;
  }
  frame_footer_.resize(VESC_FOOTER_SIZE);

  *(frame_footer_.end() - 1) = 3;
}

/**
 * @brief Constructor
 * @param frame Reference of a buffer with constant range
 * @param payload_size Specified payload size
 **/
VescFrame::VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload): payload_(payload)
{
  /* VescPacketFactory::createPacket() should make sure that
   *  the input is valid, but run a few cheap checks anyway */
  assert(boost::distance(frame) >= VESC_MIN_FRAME_SIZE);
  assert(boost::distance(frame) <= VESC_MAX_FRAME_SIZE);
  assert(boost::distance(payload) <= VESC_MAX_PAYLOAD_SIZE);
  assert(std::distance(frame.first, payload.first) > 0 && std::distance(payload.second, frame.second) > 0);

  frame_header_.resize(std::distance(frame.first, payload.first));
  frame_header_.assign(frame.first, payload.first);
  frame_footer_.resize(std::distance(payload.second, frame.second));
  frame_footer_.assign(payload.second, frame.second);
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 * @param raw Pointer of VescFrame
 **/
VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescPayload> raw) : VescData("FWVersion", raw)
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
VescPacketRequestFWVersion::VescPacketRequestFWVersion() : VescData("RequestFWVersion", 1, COMM_PACKET_ID::COMM_FW_VERSION)
{
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketValues::VescPacketValues(std::shared_ptr<VescPayload> raw) : VescData("Values", raw)
{
}

/**
 * @brief Gets temperature of MOSFETs
 * @return Temperature of MOSFETs
 **/
double VescPacketValues::getMosTemp() const
{
  return readBuffer(PACKET_MAP::TEMP_MOS, 2) / 10.0;
}

/**
 * @brief Gets temperature of the motor
 * @return Temperature of the motor
 **/
double VescPacketValues::getMotorTemp() const
{
  return readBuffer(PACKET_MAP::TEMP_MOTOR, 2) / 10.0;
}

/**
 * @brief Gets motor current
 * @return Motor current
 **/
double VescPacketValues::getMotorCurrent() const
{
  return readBuffer(PACKET_MAP::CURRENT_MOTOR, 4) / 100.0;
}

/**
 * @brief Gets input current
 * @return Input current
 **/
double VescPacketValues::getInputCurrent() const
{
  return readBuffer(PACKET_MAP::CURRENT_IN, 4) / 100.0;
}

/**
 * @brief Gets the current duty value
 * @return The current duty value
 **/
double VescPacketValues::getDuty() const
{
  int16_t duty_raw = static_cast<int32_t>(readBuffer(PACKET_MAP::DUTY_NOW, 2));

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
  return readBuffer(PACKET_MAP::ERPM, 4);
}

/**
 * @brief Gets input voltage
 * @return Input voltage
 **/
double VescPacketValues::getInputVoltage() const
{
  return readBuffer(PACKET_MAP::VOLTAGE_IN, 2) / 10.0;
}

/**
 * @brief Gets consumed charge
 * @return Consumed charge
 **/
double VescPacketValues::getConsumedCharge() const
{
  return readBuffer(PACKET_MAP::AMP_HOURS, 4) / 10000.0;
}

/**
 * @brief Gets input charge
 * @return Input charge
 **/
double VescPacketValues::getInputCharge() const
{
  return readBuffer(PACKET_MAP::AMP_HOURS_CHARGED, 4) / 10000.0;
}

/**
 * @brief Gets consumed power
 * @return Consumed power
 **/
double VescPacketValues::getConsumedPower() const
{
  return readBuffer(PACKET_MAP::TACHOMETER, 4) / 10000.0;
}

/**
 * @brief Gets input power
 * @return Input power
 **/
double VescPacketValues::getInputPower() const
{
  return readBuffer(PACKET_MAP::TACHOMETER, 4) / 10000.0;
}

/**
 * @brief Gets the current position
 * @return The current position
 **/
double VescPacketValues::getPosition() const
{
  return readBuffer(PACKET_MAP::TACHOMETER, 4);
}

/**
 * @brief Gets absolute displacement
 * @return Absolute displacement
 **/
double VescPacketValues::getDisplacement() const
{
  return readBuffer(PACKET_MAP::FAULT_CODE, 4);
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
 * @brief Reads a value from the buffer
 * @param map_id start address to read
 * @param size the number of bytes to read
 * @return Required value
 **/
double VescPacketValues::readBuffer(const PACKET_MAP  packet_map, const uint8_t size) const
{
  uint8_t map_id = static_cast<uint8_t>(packet_map);
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

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketRequestValues::VescPacketRequestValues() : VescData("RequestFWVersion", 1, COMM_PACKET_ID::COMM_GET_VALUES)
{
}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetDuty::VescPacketSetDuty(double duty) : VescData("SetDuty", 5, COMM_PACKET_ID::COMM_SET_DUTY)
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
VescPacketSetCurrent::VescPacketSetCurrent(double current) : VescData("SetCurrent", 5, COMM_PACKET_ID::COMM_SET_CURRENT)
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
  : VescData("SetCurrentBrake", 5, COMM_PACKET_ID::COMM_SET_CURRENT_BRAKE)
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
VescPacketSetVelocityERPM::VescPacketSetVelocityERPM(double vel_erpm) : VescData("SetERPM", 5, COMM_PACKET_ID::COMM_SET_ERPM)
{
  const int32_t v = static_cast<int32_t>(vel_erpm);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),3);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),4);


}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetPos::VescPacketSetPos(double pos) : VescData("SetPos", 5, COMM_PACKET_ID::COMM_SET_POS)
{
  /** @todo range check pos */
  const int32_t v = static_cast<int32_t>(pos * 100000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 24) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>((v >> 16) & 0xFF),2);
  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),3);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),4);


}

/*------------------------------------------------------------------*/

/**
 * @brief Constructor
 **/
VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos) : VescData("SetServoPos", 3, COMM_PACKET_ID::COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  uint16_t v = static_cast<uint16_t>(servo_pos * 1000.0);

  setPayloadValue(static_cast<uint8_t>((v >> 8) & 0xFF),1);
  setPayloadValue(static_cast<uint8_t>(v & 0xFF),2);

}

/*------------------------------------------------------------------*/


}  // namespace vesc_driver
