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

class VescPayload {
 public:
  virtual ~VescPayload() {}

  virtual const Buffer& getPayload() const final { return payload_; }
  explicit VescPayload(const int16_t payload_size);
  explicit VescPayload(const BufferRangeConst& payload);
  virtual const Buffer& setPayloadId(const int16_t payload_id) final {
    *payload_.begin() = payload_id;
  }
  virtual const Buffer& setPayloadValue(const int16_t payload_value,
                                        const uint8_t position) final {
    *(payload_.begin() + position) = payload_value;
  }

 protected:
  Buffer payload_;

 private:
  friend class VescPacketFactory;  // gives VescPacketFactory access to private
                                   // constructor
};

class VescData : public VescPayload {
 public:
  /**
   * @brief Destructor
   **/
  virtual ~VescData() {}

  /**
   * @brief Gets the packet name
   * @return The packet name
   **/
  virtual const std::string& getName() const final { return name_; }

 protected:
  VescData(const std::string& name, const int16_t payload_size,
           const COMM_PACKET_ID payload);
  VescData(const std::string& name, std::shared_ptr<VescPayload> raw);

 private:
  std::string name_;
};

/**
 * @brief The raw frame for communicating with the VESC
 **/
class VescFrame
{
public:
  /**
   * @brief Destructor
   **/
  virtual ~VescFrame()
  {
  }

  /**
   * @brief Gets a reference of the frame
   * @return Reference of the frame
   **/
  virtual const Buffer getFrame() const {
    Buffer frame;
    frame.clear();
    frame.insert(frame.end(), frame_header_.begin(), frame_header_.end());
    frame.insert(frame.end(), payload_.getPayload().begin(),
                 payload_.getPayload().end());
    frame.insert(frame.end(), frame_footer_.begin(), frame_footer_.end());
    return frame;
  }

  /* packet properties */
  static const int16_t VESC_MAX_PAYLOAD_SIZE = 1024;                     // Maximum payload size (bytes)
  static const int16_t VESC_MIN_FRAME_SIZE = 5;                          // Smallest frame size (bytes)
  static const int16_t VESC_MAX_FRAME_SIZE = 6 + VESC_MAX_PAYLOAD_SIZE;  // Largest frame size (bytes)
  static const int16_t VESC_SOF_VAL_SMALL_FRAME = 2;                     // Start of "small" frame value
  static const int16_t VESC_SOF_VAL_LARGE_FRAME = 3;                     // Start of "large" frame value
  static const int16_t VESC_EOF_VAL = 3;                                 // End-of-frame value

  static const int16_t VESC_MIN_HEADER_SIZE = 2;  // Minimum header size (bytes)
  static const int16_t VESC_FOOTER_SIZE = 3;      // Footer size (bytes)

  /**
   * @brief CRC parameters for the VESC
   **/
  typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> CRC;

protected:
  explicit VescFrame(const int16_t payload_size);

  Buffer frame_header_;
  Buffer frame_footer_;
  // Stores frame data

  VescPayload payload_;

private:
  VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload);
};

/*------------------------------------------------------------------*/

/**
 * @brief VescFrame with a non-zero length payload
 **/
class VescPacket : public VescFrame
{
public:
  /**
   * @brief Destructor
   **/
  virtual ~VescPacket()
  {
  }

  /**
   * @brief Gets the packet name
   * @return The packet name
   **/
  virtual const std::string& getName() const
  {
    return name_;
  }

 protected:
  VescPacket(const std::string& name, const int16_t payload_size,
             const COMM_PACKET_ID payload);
  VescPacket(const std::string& name, std::shared_ptr<VescFrame> raw);

private:
  std::string name_;
};

typedef std::shared_ptr<VescData> VescPacketPtr;
typedef std::shared_ptr<VescData const> VescPacketConstPtr;

/*------------------------------------------------------------------*/

/**
 * @brief Farmware version
 **/
class VescPacketFWVersion : public VescData
{
public:
  explicit VescPacketFWVersion(std::shared_ptr<VescPayload> raw);

  int16_t fwMajor() const;
  int16_t fwMinor() const;
};

/*------------------------------------------------------------------*/

/**
 * @brief Requests farmware version
 **/
class VescPacketRequestFWVersion : public VescData
{
public:
  VescPacketRequestFWVersion();
};

/*------------------------------------------------------------------*/

/**
 * @brief Gets values in return packets
 **/
class VescPacketValues : public VescData
{
public:
  explicit VescPacketValues(std::shared_ptr<VescPayload> raw);

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
  double getPosition() const;
  double getDisplacement() const;
  int getFaultCode() const;

private:
  double readBuffer(const PACKET_MAP, const uint8_t) const;
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for requesting retrun packets
 **/
class VescPacketRequestValues : public VescData
{
public:
  VescPacketRequestValues();
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting duty
 **/
class VescPacketSetDuty : public VescData
{
public:
  explicit VescPacketSetDuty(double duty);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting reference current
 **/
class VescPacketSetCurrent : public VescData
{
public:
  explicit VescPacketSetCurrent(double current);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting current brake
 **/
class VescPacketSetCurrentBrake : public VescData
{
public:
  explicit VescPacketSetCurrentBrake(double current_brake);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting reference angular velocity
 **/
class VescPacketSetVelocityERPM : public VescData
{
public:
  explicit VescPacketSetVelocityERPM(double vel_erpm);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting a reference position
 **/
class VescPacketSetPos : public VescData
{
public:
  explicit VescPacketSetPos(double pos);
};

/*------------------------------------------------------------------*/

/**
 * @brief Packet for setting a servo position
 **/
class VescPacketSetServoPos : public VescData
{
public:
  explicit VescPacketSetServoPos(double servo_pos);
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER_VESC_PACKET_HPP_
