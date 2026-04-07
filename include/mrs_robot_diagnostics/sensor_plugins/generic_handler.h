#pragma once

#include <mrs_robot_diagnostics/sensor_handler.h>
#include <mrs_lib/param_loader.h>
#include <rclcpp/generic_subscription.hpp>

#include <deque>
#include <mutex>

namespace mrs_robot_diagnostics
{
namespace generic_handler
{

class GenericSensorHandler : public mrs_robot_diagnostics::SensorHandler {
public:
  GenericSensorHandler() = default;

  bool initialize(rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &name_space, const std::string &topic,
                  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  mrs_msgs::msg::SensorStatus updateStatus() override;

private:
  std::string name_;
  std::string topic_;
  uint8_t     sensor_type_uint_ = 0;
  bool        is_initialized_   = false;

  // Rate monitoring
  double   expected_rate_  = 0.0;
  double   rate_tolerance_ = 0.3;
  double   measured_rate_  = -1.0;
  uint64_t msg_count_      = 0;

  // Sliding window for rate calculation
  static constexpr size_t RATE_WINDOW_SIZE = 10;
  std::deque<rclcpp::Time> msg_timestamps_;
  std::mutex               mutex_timestamps_;
  rclcpp::Time             last_msg_wall_time_;

  // Grace period before reporting rate errors
  static constexpr double GRACE_PERIOD_S = 5.0;
  rclcpp::Time            init_time_;

  // Generic subscription (type-erased)
  std::shared_ptr<rclcpp::GenericSubscription> generic_sub_;

  void    messageCallback(const std::shared_ptr<const rclcpp::SerializedMessage> &msg);
  double  calculateWindowedRate();
  uint8_t mapSensorType(const std::string &type_str);
};

}  // namespace generic_handler
}  // namespace mrs_robot_diagnostics
