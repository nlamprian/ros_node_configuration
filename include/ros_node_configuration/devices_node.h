#ifndef ROS_NODE_CONFIGURATION_DEVICES_NODE_H
#define ROS_NODE_CONFIGURATION_DEVICES_NODE_H

#include <ros/ros.h>

#include <ros_node_configuration/device/device_manager.h>

class DevicesNode {
 public:
  DevicesNode() : pnh_("~"), device_manager_(gnh_) {
    // Initialize device manager
    ros::NodeHandle devices_nh(pnh_, "devices");
    device_manager_.init(devices_nh);
  }

  ~DevicesNode() {}

 private:
  ros::NodeHandle gnh_, pnh_;

  device::DeviceManager device_manager_;
};

#endif  // ROS_NODE_CONFIGURATION_DEVICES_NODE_H
