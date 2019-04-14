#ifndef ROS_NODE_CONFIGURATION_DEVICE_DEVICE_MANAGER_H
#define ROS_NODE_CONFIGURATION_DEVICE_DEVICE_MANAGER_H

#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <ros_node_configuration/device/device_factory.h>

namespace device {

/**
 * @brief Initializes and maintains a list of devices.
 */
class DeviceManager {
 public:
  DeviceManager(ros::NodeHandle &nh) : nh_(nh) {}

  ~DeviceManager() {}

  /**
   * @brief Creates and initializes a list of devices present in the given
   * namespace.
   */
  bool init(ros::NodeHandle &devices_nh) {
    // Get device names
    ros::V_string names;
    if (not devices_nh.hasParam("names")) {
      ROS_ERROR("DeviceManager: Failed to get device names");
      return false;
    }
    devices_nh.getParam("names", names);

    // Get devices
    for (const auto &name : names) {
      ros::NodeHandle device_nh(devices_nh, name);
      DevicePtr device = DeviceFactory::get(name, nh_, device_nh);
      if (device == nullptr) {
        ROS_WARN_STREAM("DeviceManager: Failed to get device " << name);
        continue;
      }
      devices_.insert({name, device});
    }

    return true;
  }

  /**
   * @brief Get the device with the given name.
   */
  DevicePtr getDevice(const std::string &name) const {
    if (devices_.count(name) == 0) return nullptr;
    return devices_.at(name);
  }

 private:
  ros::NodeHandle nh_;

  std::unordered_map<std::string, DevicePtr> devices_;
};

using ManagerPtr = std::shared_ptr<DeviceManager>;
using ManagerConstPtr = std::shared_ptr<const DeviceManager>;

}  // namespace device

#endif  // ROS_NODE_CONFIGURATION_DEVICE_DEVICE_MANAGER_H
