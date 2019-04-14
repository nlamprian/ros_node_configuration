#ifndef ROS_NODE_CONFIGURATION_DEVICE_DEVICE_H
#define ROS_NODE_CONFIGURATION_DEVICE_DEVICE_H

#include <memory>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>

namespace device {

/**
 * @brief Device interface. All derived devices must override the `init` method.
 */
class Device {
 public:
  Device(const std::string &name, ros::NodeHandle &nh) : name_(name), nh_(nh) {}

  virtual ~Device() {}

  /**
   * @brief Gets the device name.
   */
  const std::string &getName() const { return name_; }

  /**
   * @brief Initializes the device.
   */
  virtual bool init(ros::NodeHandle &device_nh) = 0;

 protected:
  std::string name_;

  ros::NodeHandle nh_;

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

using DevicePtr = std::shared_ptr<Device>;
using DeviceConstPtr = std::shared_ptr<const Device>;

}  // namespace device

#endif  // ROS_NODE_CONFIGURATION_DEVICE_DEVICE_H
