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

namespace drivers {
namespace can_driver {

class CanPortConfig {
 public:
  /// \brief Default constructor
  CanPortConfig(std::string port) : port(port) {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strncpy(ifr_.ifr_name, port.c_str(), IFNAMSIZ);
    ioctl(socket_, SIOCGIFINDEX, &ifr_);
    if (socket_ < 0) {
      throw std::exception();
    }
    recv_addr_.can_family = AF_CAN;
    recv_addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(socket_, (struct sockaddr*)&recv_addr_, sizeof(recv_addr_)) < 0) {
      throw std::exception();
    }

    memset(&send_addr_, 0, sizeof(send_addr_));

    send_addr_.can_family = AF_CAN;
    send_addr_.can_ifindex = ifr_.ifr_ifindex;
  }

  /// \brief Function that returns the configured buad rate
  /// \returns The configured buad rate in bps
  int get_socket() const { return socket_; }

 private:
  std::string port;
  int socket_;
  struct ifreq ifr_;
  struct sockaddr_can send_addr_, recv_addr_;
};

}  // namespace can_driver
}  // namespace drivers

#endif  // CAN_DRIVER__CAN_PORT_HPP_
