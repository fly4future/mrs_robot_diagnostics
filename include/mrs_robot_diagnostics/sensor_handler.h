#pragma once
#include <deque>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_msgs/msg/sensor_info.hpp>
#include <mrs_msgs/msg/sensor_status.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mrs_robot_diagnostics
{

class SensorHandler {
public:
  bool initialize(rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &name_space, const std::string &topic,
                  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr);

  virtual mrs_msgs::msg::SensorStatus updateStatus();

  virtual ~SensorHandler() = default;

protected:
  // Hook for derived classes to do additional initialization (e.g. create subscribers) after the base class has loaded parameters and set up rate monitoring
  virtual bool onInitialize([[maybe_unused]] rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &name,
                            [[maybe_unused]] const std::string &name_space, [[maybe_unused]] const std::string &topic,
                            [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr);

  // Hook for derived classes to provide additional details in the SensorStatus message
  // By default, returns an empty JSON object, but derived classes can override this to include custom details about the sensor status (e.g. last message
  // timestamp, error counts, etc.)
  virtual std::vector<diagnostic_msgs::msg::KeyValue> fill_details();

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
  static constexpr size_t  RATE_WINDOW_SIZE = 10;
  std::deque<rclcpp::Time> msg_timestamps_;
  std::mutex               mutex_timestamps_;
  rclcpp::Time             last_msg_wall_time_;

  // Grace period before reporting rate errors
  static constexpr double GRACE_PERIOD_S = 5.0;
  rclcpp::Time            init_time_;

  mrs_lib::SubscriberHandlerOptions shopts_;
  rclcpp::QoS                       qos_profile_{10};

  // Error publisher for reporting detailed errors (optional, can be used by derived classes)
  std::shared_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_;

  enum class error_type_t : uint16_t
  {
    not_initialized,
    no_messages_received,
  };

  // | -------------------- support functions ------------------- |
  double          calculateRate(std::deque<rclcpp::Time> &timestamps);
  uint8_t         mapSensorType(const std::string &type_str);
  Eigen::Matrix3d cov2eigen(const std::array<double, 9> &msg_cov);

  template <typename MessageType>
  mrs_lib::SubscriberHandler<MessageType> create_main_subscriber(rclcpp::Node::SharedPtr &node, const std::string &topic_name,
                                                                 const rclcpp::Duration          &timeout     = mrs_lib::no_timeout,
                                                                 rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr);
};
} // namespace mrs_robot_diagnostics

#include <mrs_robot_diagnostics/create_main_subscriber.tpp>
