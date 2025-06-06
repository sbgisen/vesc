
#ifndef CAN_DRIVER__CAN_DRIVER_HPP_
#define CAN_DRIVER__CAN_DRIVER_HPP_

#include <memory>
#include <string>

#include "can_driver/can_port.hpp"
#include "io_context/io_context.hpp"

namespace drivers
{
namespace can_driver
{

class CanDriver
{
public:
  explicit CanDriver(const IoContext& ctx);

  void init_port(const std::string& device_name, const CanPortConfig& config);

  std::shared_ptr<CanPort> port() const;

private:
  const IoContext& ctx_;
  std::shared_ptr<CanPort> port_;
};

}  // namespace can_driver
}  // namespace drivers

#endif  // CAN_DRIVER__CAN_DRIVER_HPP_
