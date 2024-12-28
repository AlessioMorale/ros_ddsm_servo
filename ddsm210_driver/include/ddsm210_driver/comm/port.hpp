#ifndef DDS210_DRIVER_COMM_PORT_HPP
#define DDS210_DRIVER_COMM_PORT_HPP
#include <vector>
#include <cstdint>
#include <functional>
namespace ddsm210_driver::comm
{
using receive_callback_t = std::function<void(const std::vector<uint8_t>&)>;
class Port {
public:
  virtual ~Port() = default;
  virtual void close() = 0;
  virtual void set_receive_callback(receive_callback_t callback) = 0;
  virtual void send(const std::vector<uint8_t>& data) = 0;
};
};  // namespace ddsm210_driver::comm
#endif  // DDS210_DRIVER_COMM_PORT_HPP