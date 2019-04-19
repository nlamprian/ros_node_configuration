#ifndef ROS_NODE_CONFIGURATION_DEVICE_UTILS_SERIAL_HANDLER_H
#define ROS_NODE_CONFIGURATION_DEVICE_UTILS_SERIAL_HANDLER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <serial/serial.h>

namespace device {
namespace serial {

/**
 * @brief Connects to a serial port and offers an interface to grab incoming
 * data.
 */
class SerialHandler {
 public:
  SerialHandler(std::string port) : has_new_data_(false) {
    port = expand(port);
    try {
      serial_ = std::make_shared<::serial::Serial>(port);
    } catch (const ::serial::IOException& /*error*/) {
      ROS_ERROR_STREAM("SerialHandler: Failed to open port " << port);
      return;
    }
    ROS_INFO_STREAM("SerialHandler: Opened port " << port);

    is_active_ = true;
    data_thread_ = std::thread(&SerialHandler::dataThread, this);
  }

  ~SerialHandler() {
    is_active_ = false;
    if (data_thread_.joinable()) data_thread_.join();
  }

  /**
   * @brief Checks if the handler is actively retrieving data from the serial
   * port.
   */
  bool isActive() const { return is_active_; }

  /**
   * @brief Checks if there are new data to retrieve.
   */
  bool hasData() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return has_new_data_;
  }

  /**
   * @brief Get the latest data.
   */
  std::string getData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (not has_new_data_) return std::string();
    has_new_data_ = false;
    return data_;
  }

 protected:
  /**
   * @brief Expands a path that starts with ~ to a full path that starts with /.
   */
  std::string expand(const std::string& port) const {
    if (port[0] != '~') return port;

    std::string home(getenv("HOME"));
    if (home.empty()) {
      std::string error_msg = "$HOME variable is not set";
      ROS_ERROR("SerialHandler: %s", error_msg.c_str());
      throw std::runtime_error(error_msg);
    }

    return home + port.substr(1);
  }

  /**
   * @brief Continuously checks if there are data on the serial port and
   * retrieves them.
   */
  void dataThread() {
    while (is_active_) {
      std::unique_lock<std::mutex> lock(data_mutex_);
      try {
        if (serial_->available()) {
          data_.clear();
          serial_->readline(data_);
          data_ = data_.substr(0, data_.size() - 1);  // Get rid of \n
          has_new_data_ = true;
        }
      } catch (const ::serial::IOException& error) {
        ROS_ERROR_STREAM("SerialHandler: " << error.what());
        break;
      }
      data_cv_.wait_for(lock, std::chrono::milliseconds(5));
    }
  }

  std::shared_ptr<::serial::Serial> serial_;

  std::atomic_bool is_active_;

  bool has_new_data_;
  std::string data_;
  std::thread data_thread_;
  mutable std::mutex data_mutex_;
  std::condition_variable data_cv_;
};

using SerialHandlerPtr = std::shared_ptr<SerialHandler>;
using SerialHandlerConstPtr = std::shared_ptr<const SerialHandler>;

}  // namespace serial
}  // namespace device

#endif  // ROS_NODE_CONFIGURATION_DEVICE_UTILS_SERIAL_HANDLER_H
