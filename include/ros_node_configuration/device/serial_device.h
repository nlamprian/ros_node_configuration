#ifndef ROS_NODE_CONFIGURATION_DEVICE_SERIAL_DEVICE_H
#define ROS_NODE_CONFIGURATION_DEVICE_SERIAL_DEVICE_H

#include <std_msgs/String.h>

#include <ros_node_configuration/device/device.h>
#include <ros_node_configuration/device/utils/serial_handler.h>

namespace device {

/**
 * @brief Device that connects to a serial port and publishes the incoming data.
 */
class SerialDevice : public Device {
 public:
  SerialDevice(const std::string &name, ros::NodeHandle &nh);

  virtual ~SerialDevice() override;

  /**
   * @brief Initializes the device with the parameters in the given namespace.
   */
  virtual bool init(ros::NodeHandle &device_nh) override;

 private:
  /**
   * @brief Populates a diagnostic status message with the state of the serial
   * handler.
   */
  void getStatusDiagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &status);

  /**
   * @brief Populates a diagnostic status message with the state of the
   * published data.
   */
  void getTimeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status);

  /**
   * @brief Publishes the device data and diagnostics.
   */
  void publishTimerCallback(const ros::TimerEvent & /*event*/);

  serial::SerialHandlerPtr serial_handler_;

  ros::Timer publish_timer_;

  ros::Publisher data_publisher_;
  ros::Duration data_publish_period_;
  std_msgs::String data_msg_;
  ros::Time last_data_publish_time_;
};

using SerialDevicePtr = std::shared_ptr<SerialDevice>;
using SerialDeviceConstPtr = std::shared_ptr<const SerialDevice>;

}  // namespace device

#endif  // ROS_NODE_CONFIGURATION_DEVICE_SERIAL_DEVICE_H
