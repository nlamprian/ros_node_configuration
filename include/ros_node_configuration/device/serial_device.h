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
  SerialDevice(const std::string &name, ros::NodeHandle &nh)
      : Device(name, nh) {}

  virtual ~SerialDevice() override {}

  /**
   * @brief Initializes the device with the parameters in the given namespace.
   */
  virtual bool init(ros::NodeHandle &device_nh) override {
    // Get port
    if (not device_nh.hasParam("port")) {
      ROS_ERROR("SerialDevice[%s]: Failed to get port", name_.c_str());
      return false;
    }
    std::string port;
    device_nh.getParam("port", port);

    // Get topic name
    if (not device_nh.hasParam("topic_name")) {
      ROS_ERROR("SerialDevice[%s]: Failed to get topic_name", name_.c_str());
      return false;
    }
    std::string topic_name;
    device_nh.getParam("topic_name", topic_name);

    // Get publish rate
    if (not device_nh.hasParam("publish_rate")) {
      ROS_ERROR("SerialDevice[%s]: Failed to get publish_rate", name_.c_str());
      return false;
    }
    double publish_rate;
    device_nh.getParam("publish_rate", publish_rate);

    // Get hardware id
    if (not device_nh.hasParam("hardware_id")) {
      ROS_ERROR("SerialDevice[%s]: Failed to get hardware_id", name_.c_str());
      return false;
    }
    std::string hardware_id;
    device_nh.getParam("hardware_id", hardware_id);

    // Initialize serial handler
    serial_handler_ = std::make_shared<serial::SerialHandler>(port);

    // Initialize data topic
    data_publisher_ = nh_.advertise<std_msgs::String>(topic_name, 1);
    data_publish_period_ = ros::Duration(1 / publish_rate);

    // Initialize diagnostic updater
    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(
        nh_, device_nh, ros::this_node::getName() + ":" + name_);
    diagnostic_updater_->setHardwareID(hardware_id);
    diagnostic_updater_->add("status", this,
                             &SerialDevice::getStatusDiagnostics);
    diagnostic_updater_->add("time", this, &SerialDevice::getTimeDiagnostics);

    // Initialize publish timer
    publish_timer_ = nh_.createTimer(ros::Rate(200),
                                     &SerialDevice::publishTimerCallback, this);

    ROS_INFO("SerialDevice[%s]: Initialized successfully", name_.c_str());
    return true;
  }

 private:
  /**
   * @brief Populates a diagnostic status message with the state of the serial
   * handler.
   */
  void getStatusDiagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &status) {
    if (serial_handler_->isActive())
      status.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     "Serial handler is active");
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "Serial port is not active");
  }

  /**
   * @brief Populates a diagnostic status message with the state of the
   * published data.
   */
  void getTimeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    ros::Duration time = ros::Time::now() - last_data_publish_time_;
    if (time < ros::Duration(1))
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    else
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No recent data");

    stat.add("Time since last data", time.toSec());
  }

  /**
   * @brief Publishes the device data and diagnostics.
   */
  void publishTimerCallback(const ros::TimerEvent & /*event*/) {
    // Publish data
    if (serial_handler_->hasData() and
        ros::Time::now() - last_data_publish_time_ >= data_publish_period_) {
      data_msg_.data = serial_handler_->getData();
      data_publisher_.publish(data_msg_);
      last_data_publish_time_ = ros::Time::now();
    }

    // Publish diagnostics
    diagnostic_updater_->update();
  }

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
