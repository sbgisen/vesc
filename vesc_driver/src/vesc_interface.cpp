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

#include "vesc_driver/vesc_interface.hpp"

#include <can_driver/can_driver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

namespace vesc_driver {
class VescInterface::Impl {
 public:
  Impl()
      : owned_ctx(new IoContext(2)),
        serial_driver_(new drivers::serial_driver::SerialDriver(*owned_ctx)),
        can_driver_(new drivers::can_driver::CanDriver(*owned_ctx)) {
    data_updated_ = false;
  }

  void* rxThread(void);

  void* canThread(void);

  static void* rxThreadHelper(void* context) {
    return ((VescInterface::Impl*)context)->rxThread();
  }

  static void* canThreadHelper(void* context) {
    return ((VescInterface::Impl*)context)->canThread();
  }

  pthread_t rx_thread_;
  bool rx_thread_run_;
  bool can_thread_run_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  std::unique_ptr<IoContext> owned_ctx{};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::can_driver::CanPortConfig> can_config_;
  CRC send_crc_;
  bool data_updated_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::unique_ptr<drivers::can_driver::CanDriver> can_driver_;
};

void* VescInterface::Impl::canThread(void) {
  while (can_thread_run_) {
    Buffer tmp_buffer;
    uint32_t header = 0;

    can_driver_->port()->receive(tmp_buffer, header);
    CAN_PACKET_ID cmd = static_cast<CAN_PACKET_ID>(header >> 8);  // command
    Buffer buffer(0);
    switch (cmd) {
      case CAN_PACKET_ID::CAN_PACKET_PROCESS_SHORT_BUFFER: {  // end of packet
        uint32_t ind = 0;
        // const uint32_t controller_id = tmp_buffer[ind++];
        // const int rx_buffer_response_type = tmp_buffer[ind++];
        ind += 2;
        const unsigned int len =
            static_cast<unsigned int>(tmp_buffer.size()) - ind;
        if (len > 6) {
          error_handler_(
              "CAN_PACKET_PROCESS_SHORT_BUFFER should be smaller than 7 but we "
              "got " +
              std::to_string(len));
        }
        buffer.insert(buffer.end(), tmp_buffer.begin() + ind, tmp_buffer.end());

        std::string error;
        int bytes_needed = VESC_MIN_FRAME_SIZE;
        VescPacketConstPtr packet = VescPacketFactory::createCanPacket(
            buffer.begin(), buffer.end(), &bytes_needed, &error);
        if (packet) {
          data_updated_ = true;
          packet_handler_(packet);
        }
        buffer.erase(buffer.begin(), buffer.end());

      } break;
      case CAN_PACKET_ID::CAN_PACKET_FILL_RX_BUFFER: {
        uint32_t ind = 0;
        ind++;
        buffer.insert(buffer.end(), tmp_buffer.begin() + ind, tmp_buffer.end());

      }

      break;
      case CAN_PACKET_ID::CAN_PACKET_FILL_RX_BUFFER_LONG: {
        uint32_t ind = 0;
        // unsigned int packet_number =
        //     tmp_buffer[ind++] + (tmp_buffer[ind++] << 8);
        ind += 2;
        buffer.insert(buffer.end(), tmp_buffer.begin() + ind, tmp_buffer.end());

      } break;

      case CAN_PACKET_ID::CAN_PACKET_PROCESS_RX_BUFFER: {  // end of packet
        if (tmp_buffer.size() != 6) {
          error_handler_(
              "CAN_PCAKET_PROCESS_RX_BUFFER should be equal 6 but we got " +
              std::to_string(tmp_buffer.size()));
        }
        int ind = 0;
        // const uint8_t controller_id = tmp_buffer[ind++];
        // const unsigned int rx_buffer_response_type = tmp_buffer[ind++];
        ind += 2;
        const unsigned int full_data_len =
            (tmp_buffer[ind] << 8) + tmp_buffer[ind + 1];
        ind += 2;
        // const uint16_t crc = static_cast<uint16_t>(tmp_buffer[ind++])
        //                      << 8 + tmp_buffer[ind++];
        // TODO: check crc
        // if (crc != crc_calc.checksum()) {
        //   error_handler_("Invalid checksum");
        // }

        if (full_data_len != buffer.size()) {
          error_handler_("Invalid data length");
        }

        std::string error;
        int bytes_needed = VESC_MIN_FRAME_SIZE;
        VescPacketConstPtr packet = VescPacketFactory::createCanPacket(
            buffer.begin(), buffer.end(), &bytes_needed, &error);
        if (packet) {
          data_updated_ = true;
          packet_handler_(packet);
        }
        buffer.erase(buffer.begin(), buffer.end());
      }
      default:
        break;
    }
    if (cmd == CAN_PACKET_ID::CAN_PACKET_PROCESS_SHORT_BUFFER) {
    } else if (cmd == CAN_PACKET_ID::CAN_PACKET_FILL_RX_BUFFER) {
    }
  }
  return NULL;
}

void* VescInterface::Impl::rxThread(void) {
  Buffer buffer;
  buffer.reserve(4096);
  auto temp_buffer = Buffer(4096);

  while (rx_thread_run_) {
    int bytes_needed = VESC_MIN_FRAME_SIZE;
    // attempt to read at least bytes_needed bytes from the serial port
    const auto bytes_read = serial_driver_->port()->receive(temp_buffer);
    buffer.reserve(buffer.size() + bytes_read);
    buffer.insert(buffer.end(), temp_buffer.begin(),
                  temp_buffer.begin() + bytes_read);

    if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty()) {
      error_handler_(
          "Possibly out-of-sync with VESC, read timout in the middle of a "
          "frame.");
    }
    if (!buffer.empty()) {
      // search buffer for valid packet(s)
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while (iter != buffer.end()) {
        // check if valid start-of-frame character
        if (VESC_SOF_VAL_SMALL_FRAME == *iter ||
            VESC_SOF_VAL_LARGE_FRAME == *iter) {
          // good start, now attempt to create packet
          std::string error;
          int frame_size = 0;
          VescPacketConstPtr packet = VescPacketFactory::createPacket(
              iter, buffer.end(), &bytes_needed, &frame_size, &error);
          if (packet) {
            // Packet received;
            data_updated_ = true;
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0) {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. "
                    "Discarding "
                 << std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + frame_size;
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          } else if (bytes_needed > 0) {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          } else {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end()) bytes_needed = VESC_MIN_FRAME_SIZE;

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0) {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding "
           << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }
  }
  return NULL;
}

VescInterface::VescInterface(const std::string& port, const int& controller_id,
                             const int& vesc_id,
                             const PacketHandlerFunction& packet_handler,
                             const ErrorHandlerFunction& error_handler)
    : impl_(new Impl()), port_(port) {
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty()) connect(port, controller_id, vesc_id);
}

VescInterface::~VescInterface() {
  // stops the motor
  setDutyCycle(0.0);

  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction& handler) {
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction& handler) {
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string& port, const int& controller_id,
                            const int& vesct_id) {
  // todo - mutex?
  port_ = port;
  std::string usb_port = "/dev/tty";
  std::string can_port = "can";
  if (std::equal(usb_port.begin(), usb_port.end(), port.begin())) {
    if (isConnected()) {
      throw SerialException("Already connected to serial port.");
    }
    // connect to serial port
    try {
      const uint32_t baud_rate = 115200;
      const auto fc = drivers::serial_driver::FlowControl::NONE;
      const auto pt = drivers::serial_driver::Parity::NONE;
      const auto sb = drivers::serial_driver::StopBits::ONE;
      impl_->device_config_ =
          std::make_unique<drivers::serial_driver::SerialPortConfig>(
              baud_rate, fc, pt, sb);
      impl_->serial_driver_->init_port(port, *impl_->device_config_);
      if (!impl_->serial_driver_->port()->is_open()) {
        impl_->serial_driver_->port()->open();
      }
    } catch (const std::exception& e) {
      std::stringstream ss;
      ss << "Failed to open the serial port to the VESC. " << e.what();
      throw SerialException(ss.str().c_str());
    }

    // start up a monitoring thread
    impl_->rx_thread_run_ = true;
    int result =
        pthread_create(&impl_->rx_thread_, NULL,
                       &VescInterface::Impl::rxThreadHelper, impl_.get());
    assert(0 == result);

  } else if (std::equal(can_port.begin(), can_port.end(), port.begin())) {
    // connect to can port
    try {
      impl_->can_config_ = std::make_unique<drivers::can_driver::CanPortConfig>(
          port, controller_id, vesct_id);

      impl_->can_driver_->init_port(port, *impl_->can_config_);

      impl_->can_thread_run_ = true;

      int result =
          pthread_create(&impl_->rx_thread_, NULL,
                         &VescInterface::Impl::canThreadHelper, impl_.get());

      assert(0 == result);

    } catch (const std::exception& e) {
      std::stringstream ss;
      ss << "Failed to open the can port to the VESC. " << e.what();
      throw SerialException(ss.str().c_str());
    }
  } else {
    throw SerialException("Invalid port name.");
  }
}

void VescInterface::disconnect() {
  // todo - mutex?
  const std::string usb_port = "/dev/tty";
  const std::string can_port = "can";
  if (isConnected()) {
    // bring down read thread
    if (std::equal(usb_port.begin(), usb_port.end(), port_.begin())) {
      impl_->rx_thread_run_ = false;
      int result = pthread_join(impl_->rx_thread_, NULL);
      assert(0 == result);
      impl_->serial_driver_->port()->close();
      return;
    }
    if (std::equal(can_port.begin(), can_port.end(), port_.begin())) {
      impl_->can_thread_run_ = false;
      int result = pthread_join(impl_->rx_thread_, NULL);
      assert(0 == result);
      return;
    }
  }
}

bool VescInterface::isConnected() const {
  const std::string usb_port = "/dev/tty";
  const std::string can_port = "can";
  if (std::equal(usb_port.begin(), usb_port.end(), port_.begin())) {
    auto port = impl_->serial_driver_->port();

    if (port) {
      return port->is_open();
    } else {
      return false;
    }
  } else if (std::equal(can_port.begin(), can_port.end(), port_.begin())) {
    auto port = impl_->can_driver_->port();

    if (port) {
      return true;
    } else {
      return false;
    }
  } else {
    impl_->error_handler_("Unknown port type.");
    return false;
  }
}

bool VescInterface::isRxDataUpdated() const {
  bool output = impl_->data_updated_;
  impl_->data_updated_ = false;
  return output;
}

void VescInterface::send(const VescPacket& data) {
  std::string usb_port = "/dev/tty";
  std::string can_port = "can";
  if (std::equal(usb_port.begin(), usb_port.end(), port_.begin())) {
    Buffer frame;
    frame.clear();
    int16_t payload_size = data.getPayload().size();
    // header
    assert(payload_size >= 0 && payload_size <= 1024);

    if (payload_size < 256) {
      // single byte payload size
      frame.push_back(static_cast<uint8_t>(VESC_SOF_VAL_SMALL_FRAME));
      frame.push_back(static_cast<uint8_t>(payload_size));

    } else {
      // two byte payload size
      frame.push_back(static_cast<uint8_t>(VESC_SOF_VAL_LARGE_FRAME));
      frame.push_back(static_cast<uint8_t>(payload_size >> 8));
      frame.push_back(static_cast<uint8_t>(payload_size & 0xFF));
    }

    // payload
    frame.insert(frame.end(), data.getPayload().begin(),
                 data.getPayload().end());
    // calculate CRC
    CRC crc_calc;
    crc_calc.process_bytes(&(*(data.getPayload().begin())),
                           boost::distance(data.getPayload()));
    uint16_t crc = crc_calc.checksum();
    frame.push_back(static_cast<uint8_t>(crc >> 8));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>(VESC_EOF_VAL));

    std::size_t written = impl_->serial_driver_->port()->send(frame);
    if (written != frame.size()) {
      std::stringstream ss;
      ss << "Wrote " << written << " bytes, expected " << frame.size() << ".";
      throw SerialException(ss.str().c_str());
    }
  } else if (std::equal(can_port.begin(), can_port.end(), port_.begin())) {
    int len = data.getPayload().size();
    Buffer buffer(0);

    if (len <= 6) {
      buffer.push_back(6);
      buffer.push_back(0);
      buffer.insert(buffer.end(), data.getPayload().begin(),
                    data.getPayload().end());
      uint32_t header =
          (static_cast<uint32_t>(CAN_PACKET_ID::CAN_PACKET_PROCESS_SHORT_BUFFER)
           << 8);
      impl_->can_driver_->port()->send(buffer, header);

    } else {
      unsigned int end_a = 0;
      for (int i = 0; i < len; i += 7) {
        if (i > 255) {
          break;
        }

        end_a = i + 7;

        uint8_t send_len = 7;
        buffer.push_back(i);

        if ((i + 7) <= len) {
          buffer.insert(buffer.end(), data.getPayload().begin() + i,
                        data.getPayload().begin() + i + send_len);
        } else {
          buffer.insert(buffer.end(), data.getPayload().begin() + i,
                        data.getPayload().end());
        }
        uint32_t header =
            (static_cast<uint32_t>(CAN_PACKET_ID::CAN_PACKET_FILL_RX_BUFFER)
             << 8);

        impl_->can_driver_->port()->send(buffer, header);
      }

      for (int i = end_a; i < len; i += 6) {
        uint8_t send_len = 6;
        buffer.push_back(i >> 8);
        buffer.push_back(i & 0xFF);

        if ((i + 6) <= len) {
          buffer.insert(buffer.end(), data.getPayload().begin() + i,
                        data.getPayload().begin() + i + send_len);
        } else {
          buffer.insert(buffer.end(), data.getPayload().begin() + i,
                        data.getPayload().end());
        }

        uint32_t header = (static_cast<uint32_t>(
                               CAN_PACKET_ID::CAN_PACKET_FILL_RX_BUFFER_LONG)
                           << 8);
        impl_->can_driver_->port()->send(buffer, header);
      }
      buffer.push_back(6);
      buffer.push_back(0);
      buffer.push_back(len >> 8);
      buffer.push_back(len & 0xFF);
      CRC crc_calc;
      crc_calc.process_bytes(&(*(data.getPayload().begin())),
                             boost::distance(data.getPayload()));
      uint16_t crc = crc_calc.checksum();
      buffer.push_back((uint8_t)(crc >> 8));
      buffer.push_back((uint8_t)(crc & 0xFF));

      uint32_t header =
          (static_cast<uint32_t>(CAN_PACKET_ID::CAN_PACKET_PROCESS_RX_BUFFER)
           << 8);
      impl_->can_driver_->port()->send(buffer, header);
    }
  }
}

void VescInterface::requestFWVersion() { send(VescPacketRequestFWVersion()); }

void VescInterface::requestState() { send(VescPacketRequestValues()); }

void VescInterface::setDutyCycle(double duty_cycle) {
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current) {
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake) {
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed) {
  send(VescPacketSetVelocityERPM(speed));
}

void VescInterface::setPosition(double position) {
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo) {
  send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver
