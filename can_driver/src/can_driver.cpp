#include "can_driver/can_driver.hpp"

#include <memory>
#include <string>

namespace drivers {
namespace can_driver {

CanDriver::CanDriver(const IoContext& ctx) : ctx_(ctx) {}

void CanDriver::init_port(const std::string& device_name,
                          const CanPortConfig& config) {
  port_.reset(new CanPort(ctx_, device_name, config));
}

std::shared_ptr<CanPort> CanDriver::port() const { return port_; }

}  // namespace can_driver
}  // namespace drivers
