#ifndef ROS_NODE_CONFIGURATION_DEVICE_TYPES_H
#define ROS_NODE_CONFIGURATION_DEVICE_TYPES_H

#include <cstdint>
#include <string>
#include <type_traits>

/**
 * @brief Returns the underlying value of an enumeration value.
 */
template <typename Enumeration>
auto as_integer(Enumeration const value) ->
    typename std::underlying_type<Enumeration>::type {
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

namespace device {

enum class DeviceType : uint8_t { INVALID = 0, SERIAL_DEVICE = 1 };

/**
 * @brief Turns a string type to a DeviceType.
 */
static DeviceType parseType(const std::string &type) {
  if (type == "serial") {
    return DeviceType::SERIAL_DEVICE;
  } else {
    return DeviceType::INVALID;
  }
}

}  // namespace device

#endif  // ROS_NODE_CONFIGURATION_DEVICE_TYPES_H
