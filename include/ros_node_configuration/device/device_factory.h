#ifndef ROS_NODE_CONFIGURATION_DEVICE_DEVICE_FACTORY_H
#define ROS_NODE_CONFIGURATION_DEVICE_DEVICE_FACTORY_H

#include <ros_node_configuration/device/serial_device.h>
#include <ros_node_configuration/device/types.h>

namespace device {

/**
 * @brief Creates devices.
 */
struct DeviceFactory {
  static DevicePtr get(const std::string &name, ros::NodeHandle &root_nh,
                       ros::NodeHandle &device_nh) {
    // Get type
    if (not device_nh.hasParam("type")) {
      ROS_ERROR("DeviceFactory: Failed to get device type");
      return nullptr;
    }
    std::string type;
    device_nh.getParam("type", type);

    // Get device
    return get(name, parseType(type), root_nh, device_nh);
  }

  static DevicePtr get(const std::string &name, DeviceType type,
                       ros::NodeHandle &root_nh, ros::NodeHandle &device_nh) {
    DevicePtr device;

    switch (type) {
      case DeviceType::SERIAL_DEVICE:
        device = std::make_shared<SerialDevice>(name, root_nh);
        break;
      default:
        throw std::runtime_error("Requested invalid device type " +
                                 std::to_string(as_integer(type)));
    }

    bool inited = device->init(device_nh);
    if (not inited) return nullptr;

    return device;
  }

  DeviceFactory() = delete;
};

}  // namespace device

#endif  // ROS_NODE_CONFIGURATION_DEVICE_DEVICE_FACTORY_H
