#ifndef CAN_DRIVER__CAN_PORT_HPP_
#define CAN_DRIVER__CAN_PORT_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <array>
#include <cstring>
#include <string>
#include <vector>

#include "io_context/common.hpp"
#include "io_context/io_context.hpp"
using drivers::common::IoContext;

namespace drivers
{
namespace can_driver
{

using Functor = std::function<void(std::vector<uint8_t>&, const size_t&)>;

class CanPortConfig
{
public:
  /// \brief Default constructor
  CanPortConfig(const std::string& port, const int& controller_id, const int& vesc_id)
    : port_(port), controller_id_(controller_id), vesct_id_(vesc_id)
  {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strncpy(ifr_.ifr_name, port.c_str(), IFNAMSIZ);
    ioctl(socket_, SIOCGIFINDEX, &ifr_);
    if (socket_ < 0)
    {
      throw std::exception();
    }
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(socket_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
    {
      throw std::exception();
    }
  }

  int get_socket() const
  {
    return socket_;
  }
  const int& get_controller_id() const
  {
    return controller_id_;
  }
  const int& get_vesct_id() const
  {
    return vesct_id_;
  }
  struct sockaddr_can addr_;

private:
  const std::string& port_;
  int socket_;
  struct ifreq ifr_;

  const int &controller_id_, &vesct_id_;
};

class CanPort
{
public:
  CanPort(const IoContext& ctx, const std::string& device_name, const CanPortConfig can_port_config);
  ~CanPort();

  CanPort(const CanPort&) = delete;
  CanPort& operator=(const CanPort&) = delete;
  std::string device_name() const;

  CanPortConfig can_port_config() const;

  size_t send(const std::vector<uint8_t>& buff, const uint32_t& header = 0);

  size_t receive(std::vector<uint8_t>& buff, uint32_t& header);

  void async_send(const std::vector<uint8_t>& buff);

  void async_receive(Functor func);

  bool send_break();

private:
  void async_send_handler(const asio::error_code& error, size_t bytes_transferred);

  void async_receive_handler(const asio::error_code& error, size_t bytes_transferred);

  const IoContext& ctx_;
  std::string device_name_;
  asio::posix::basic_stream_descriptor<> can_port_;
  CanPortConfig port_config_;
  Functor func_;
  static constexpr size_t recv_buffer_size_{ 2048 };
  struct can_frame recv_frame_;
  uint32_t header_;
};

}  // namespace can_driver
}  // namespace drivers

#endif  // CAN_DRIVER__CAN_PORT_HPP_
