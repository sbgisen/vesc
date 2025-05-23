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

#ifndef VESC_DRIVER_VESC_PACKET_HPP_
#define VESC_DRIVER_VESC_PACKET_HPP_

#include <boost/crc.hpp>
#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vesc_driver/data_map.hpp"

namespace vesc_driver
{
typedef std::vector<uint8_t> Buffer;
typedef std::pair<Buffer::iterator, Buffer::iterator> BufferRange;
typedef std::pair<Buffer::const_iterator, Buffer::const_iterator> BufferRangeConst;

class VescFrame {
 public:
  virtual ~VescFrame() {}  // segmenation fault

  virtual const Buffer& getPayload() const final { return payload_; }
  explicit VescFrame(const int16_t payload_size);
  explicit VescFrame(const BufferRangeConst& payload);
  virtual void setPayloadId(const int16_t payload_id) final {
    *payload_.begin() = payload_id;
  }
  virtual void setPayloadValue(const int16_t payload_value,
                               const uint8_t position) final {
    *(payload_.begin() + position) = payload_value;
  }

 protected:
  Buffer payload_;

 private:
  friend class VescPacketFactory;  // gives VescPacketFactory access to private
                                   // constructor
};

class VescPacket : public VescFrame {
 public:
  /**
   * @brief Destructor
   **/
  virtual ~VescPacket() {}

  /**
   * @brief Gets the packet name
   * @return The packet name
   **/
  virtual const std::string& getName() const final { return name_; }

 protected:
  VescPacket(const std::string& name, const int16_t payload_size,
           const COMM_PACKET_ID cmd);
  VescPacket(const std::string& name, std::shared_ptr<VescFrame> raw);
  double readBuffer(const uint8_t, const uint8_t) const;
  double readAutoBuffer(const int) const;

 private:
  std::string name_;
};


class VescCanPacket : public VescFrame{
  public:
    virtual ~VescCanPacket() {}
    virtual const std::string& getName() const final { return name_; }
    virtual const CAN_PACKET_ID& getCanPacketId() const final { return can_packet_id_; }
  protected:
    VescCanPacket(const std::string& name, const int16_t payload_size, const CAN_PACKET_ID cmd);
    VescCanPacket(const std::string& name, std::shared_ptr<VescFrame> raw);
  private:
    std::string name_;  
    CAN_PACKET_ID can_packet_id_;
};
/*------------------------------------------------------------------*/

/**
 * @brief VescFrame with a non-zero length payload
 **/


typedef std::shared_ptr<VescPacket> VescPacketPtr;
typedef std::shared_ptr<VescPacket const> VescPacketConstPtr;

/*------------------------------------------------------------------*/

/**
 * @brief Firmware version
 **/
class VescPacketFWVersion : public VescPacket
{
public:
  explicit VescPacketFWVersion(std::shared_ptr<VescFrame> raw);

  int16_t fwMajor() const;
  int16_t fwMinor() const;
};

/*------------------------------------------------------------------*/

/**
 * @brief Requests firmware version
 **/
class VescPacketRequestFWVersion : public VescPacket
{
public:
  VescPacketRequestFWVersion();
};

/*------------------------------------------------------------------*/

/**
 * @brief Gets values in COMM_GET_VALUES return packets
 **/
class VescPacketValues : public VescPacket
{
public:
  explicit VescPacketValues(std::shared_ptr<VescFrame> raw);

  double getMosTemp() const;
  double getMotorTemp() const;
  double getMotorCurrent() const;
  double getInputCurrent() const;
  double getVelocityERPM() const;
  double getInputVoltage() const;
  double getDuty() const;
  double getConsumedCharge() const;
  double getInputCharge() const;
  double getConsumedPower() const;
  double getInputPower() const;
  double getTachometer() const;
  double getDisplacement() const;
  int getFaultCode() const;
  double getPosition() const;
  int getControllerID() const;
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for requesting COMM_GET_VALUES return packets
 **/
class VescPacketRequestValues : public VescPacket
{
public:
  VescPacketRequestValues();
};

/*------------------------------------------------------------------*/

/**
 * @brief Gets values in COMM_GET_MCCONF return packets
 **/
class VescPacketMCConf : public VescPacket
{
public:
  explicit VescPacketMCConf(std::shared_ptr<VescFrame> raw);

  MCConfiguration getConfig() const;

private:
  MCConfiguration config_;
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for requesting COMM_GET_MCCONF return packets
 **/
class VescPacketRequestMCConf : public VescPacket
{
public:
  VescPacketRequestMCConf();
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting duty
 **/
class VescPacketSetDuty : public VescPacket
{
public:
  explicit VescPacketSetDuty(double duty);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting reference current
 **/
class VescPacketSetCurrent : public VescPacket
{
public:
  explicit VescPacketSetCurrent(double current);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting current brake
 **/
class VescPacketSetCurrentBrake : public VescPacket
{
public:
  explicit VescPacketSetCurrentBrake(double current_brake);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting reference angular velocity
 **/
class VescPacketSetVelocityERPM : public VescPacket
{
public:
  explicit VescPacketSetVelocityERPM(double vel_erpm);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting a reference position
 **/
class VescPacketSetPos : public VescPacket
{
public:
  explicit VescPacketSetPos(double pos);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting a servo position
 **/
class VescPacketSetServoPos : public VescPacket
{
public:
  explicit VescPacketSetServoPos(double servo_pos);
};



/**
 * @brief Packet for setting duty
 **/
class VescCanPacketSetDuty : public VescCanPacket
{
public:
  explicit VescCanPacketSetDuty(double duty);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting reference current
 **/
class VescCanPacketSetCurrent : public VescCanPacket
{
public:
  explicit VescCanPacketSetCurrent(double current);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting current brake
 **/
class VescCanPacketSetCurrentBrake : public VescCanPacket
{
public:
  explicit VescCanPacketSetCurrentBrake(double current_brake);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting reference angular velocity
 **/
class VescCanPacketSetVelocityERPM : public VescCanPacket
{
public:
  explicit VescCanPacketSetVelocityERPM(double vel_erpm);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting a reference position
 **/
class VescCanPacketSetPos : public VescCanPacket
{
public:
  explicit VescCanPacketSetPos(double pos);
};

/*------------------------------------------------------------------*/


}  // namespace vesc_driver

#endif  // VESC_DRIVER_VESC_PACKET_HPP_
