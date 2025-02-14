#include "can_driver/can_port.hpp"

#include <rclcpp/logging.hpp>
#include <string>
#include <utility>
#include <vector>

namespace drivers {
namespace can_driver {

CanPort::CanPort(const IoContext& ctx, const std::string& device_name,
                 const CanPortConfig can_port_config)
    : ctx_(ctx),
      device_name_(device_name),
      can_port_(ctx.ios()),
      port_config_(can_port_config) {
  // recv_frame_.resize(recv_buffer_size_);
  can_port_.assign(port_config_.get_socket());
}

CanPort::~CanPort() {}

size_t CanPort::send(const std::vector<uint8_t>& buff, const uint32_t& header) {
  struct can_frame frame;
  frame.can_id = port_config_.get_vesct_id() | CAN_EFF_FLAG |
                 (static_cast<uint32_t>(header) << 8);
  frame.can_dlc = buff.size();
  frame.len = buff.size();
  std::memcpy(frame.data, buff.data(), buff.size());
  return can_port_.write_some(asio::buffer(&frame, sizeof(frame)));
}

size_t CanPort::receive(std::vector<uint8_t>& buff, uint32_t& header) {
  struct can_frame frame;
  can_port_.read_some(asio::mutable_buffer(&frame, sizeof(frame)));
  uint32_t eid = frame.can_id & CAN_EFF_MASK;
  uint8_t id = eid & 0xFF;
  header = eid >> 8;
  if (id != port_config_.get_controller_id()) {
    return 0;
  }
  buff.erase(buff.begin(), buff.end());
  buff.insert(buff.end(), frame.data, frame.data + frame.can_dlc);

  return frame.can_dlc;
}

void CanPort::async_send(const std::vector<uint8_t>& buff) {
  struct can_frame frame;
  frame.can_id = port_config_.get_vesct_id() | CAN_EFF_FLAG;
  frame.can_dlc = buff.size();
  frame.len = buff.size();
  can_port_.async_write_some(
      asio::buffer(&frame, sizeof(frame)),
      [this](std::error_code error, size_t bytes_transferred) {
        async_send_handler(error, bytes_transferred);
      });
}

void CanPort::async_receive(Functor func) {
  func_ = std::move(func);
  can_port_.async_read_some(
      asio::buffer(&recv_frame_, sizeof(recv_frame_)),
      [this](std::error_code error, size_t bytes_transferred) {
        async_receive_handler(error, bytes_transferred);
      });
}

// bool CanPort::send_break() {
//   bool break_sent = false;
//   if (is_open()) {
//     m_can_port.send_break();
//     break_sent = true;
//   }
//   return break_sent;
// }

void CanPort::async_send_handler(const asio::error_code& error,
                                 size_t bytes_transferred) {
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("CanPort::async_send_handler"),
                        error.message());
    return;
  }
}

void CanPort::async_receive_handler(const asio::error_code& error,
                                    size_t bytes_transferred) {
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("CanPort::async_receive_handler"),
                        error.message());
    return;
  }

  if (bytes_transferred > 0 && func_) {
    std::vector<uint8_t> recv_buffer(0);
    std::copy(recv_frame_.data, recv_frame_.data + recv_frame_.can_dlc,
              std::back_inserter(recv_buffer));
    func_(recv_buffer, bytes_transferred);
    can_port_.async_read_some(
        asio::buffer(&recv_frame_, sizeof(recv_frame_)),
        [this](std::error_code error, size_t bytes_transferred) {
          async_receive_handler(error, bytes_transferred);
        });
  }
}

std::string CanPort::device_name() const { return device_name_; }

CanPortConfig CanPort::can_port_config() const { return port_config_; }

}  // namespace can_driver
}  // namespace drivers
