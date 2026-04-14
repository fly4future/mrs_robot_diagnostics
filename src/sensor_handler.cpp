#include <mrs_robot_diagnostics/sensor_handler.h>

namespace mrs_robot_diagnostics
{

bool SensorHandler::initialize(rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &name_space, const std::string &topic,
                               rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {
  name_  = name;
  topic_ = topic;

  std::string handler_instance = name + " handler (" + topic + ")";

  error_publisher_ = std::make_shared<mrs_lib::errorgraph::ErrorPublisher>(node, node->get_clock(), "StateMonitor", handler_instance);
  mrs_lib::ParamLoader param_loader(node, "SensorHandler");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.setPrefix("robot_diagnostics/sensor_handlers/");

  std::string sensor_type_str;
  param_loader.loadParam(name + "/expected_rate", expected_rate_);
  param_loader.loadParam(name + "/rate_tolerance", rate_tolerance_, 0.3);
  param_loader.loadParam(name + "/type", sensor_type_str);

  std::string qos_reliability;
  param_loader.loadParam(name + "/qos_reliability", qos_reliability, std::string("reliable"));

  if (param_loader.loadedSuccessfully()) {
    RCLCPP_INFO(node->get_logger(), "[SensorHandler] Successfully loaded config for topic '%s'", topic.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "[SensorHandler] Failed to load config for generic handler '%s', not initializing", name.c_str());
    error_publisher_->addOneshotError("Failed to load config for sensor handler " + name_);
    return false;
  }

  sensor_type_uint_ = mapSensorType(sensor_type_str);

  // Create QoS profile based on config
  qos_profile_ = rclcpp::QoS(10);
  if (qos_reliability == "best_effort") {
    qos_profile_.best_effort();
  } else {
    qos_profile_.reliable();
  }
  qos_profile_.durability_volatile();

  // Initialize timing
  init_time_          = rclcpp::Clock(RCL_STEADY_TIME).now();
  last_msg_wall_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

  is_initialized_ = true;

  return onInitialize(node, name, name_space, topic, cbkgrp_subs);
}

mrs_msgs::msg::SensorStatus SensorHandler::updateStatus() {
  mrs_msgs::msg::SensorStatus ss;
  ss.name  = name_;
  ss.type  = sensor_type_uint_;
  ss.topic = topic_;

  if (!is_initialized_) {
    ss.ready   = false;
    ss.rate    = -1.0;
    ss.message = "Not initialized";
    ss.level   = mrs_msgs::msg::SensorStatus::ERROR;
    error_publisher_->addGeneralError(error_type_t::not_initialized, "Sensor handler " + name_ + " is not initialized");
    return ss;
  }

  std::scoped_lock lock(mutex_timestamps_);

  rclcpp::Time now                = rclcpp::Clock(RCL_STEADY_TIME).now();
  double       elapsed_since_init = (now - init_time_).seconds();

  // Grace period — don't report rate errors right after startup
  if (elapsed_since_init < GRACE_PERIOD_S) {
    ss.ready   = false;
    ss.rate    = measured_rate_;
    ss.message = "Initializing (grace period)";
    ss.level   = mrs_msgs::msg::SensorStatus::STALE;
    return ss;
  }

  // No messages ever received
  if (msg_count_ == 0) {
    ss.ready   = false;
    ss.rate    = 0.0;
    ss.message = "No messages received yet";
    ss.level   = mrs_msgs::msg::SensorStatus::ERROR;
    error_publisher_->addGeneralError(error_type_t::no_messages_received, "No messages received on topic " + topic_ + " since startup");
    return ss;
  }

  // Topic gone silent — no message for 3x the expected period
  double time_since_last = (now - last_msg_wall_time_).seconds();
  double expected_period = 1.0 / expected_rate_;

  if (time_since_last > expected_period * 3.0) {
    ss.ready   = false;
    ss.rate    = 0.0;
    ss.message = "No messages received for " + std::to_string(time_since_last) + " seconds";
    ss.level   = mrs_msgs::msg::SensorStatus::ERROR;
    error_publisher_->addGeneralError(error_type_t::no_messages_received,
                                      "No messages received on topic " + topic_ + " for " + std::to_string(time_since_last) + " seconds");
    return ss;
  }

  // Rate comparison
  ss.rate = measured_rate_;

  double lower_bound = expected_rate_ * (1.0 - rate_tolerance_);
  double upper_bound = expected_rate_ * (1.0 + rate_tolerance_);

  if (measured_rate_ >= lower_bound && measured_rate_ <= upper_bound) {
    ss.ready   = true;
    ss.level   = mrs_msgs::msg::SensorStatus::OK;
    ss.message = "Rate within expected range";
  } else if (measured_rate_ < lower_bound) {
    ss.ready   = false;
    ss.level   = mrs_msgs::msg::SensorStatus::WARN;
    ss.message = "Rate too low: expected " + std::to_string(expected_rate_) + " Hz, got " + std::to_string(measured_rate_) + " Hz";
  } else {
    ss.ready   = true;
    ss.level   = mrs_msgs::msg::SensorStatus::WARN;
    ss.message = "Rate too high: expected " + std::to_string(expected_rate_);
  }

  ss.details = fill_details();

  return ss;
}

bool SensorHandler::onInitialize([[maybe_unused]] rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &name,
                                 [[maybe_unused]] const std::string &name_space, [[maybe_unused]] const std::string &topic,
                                 [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {
  return true;
}

std::vector<diagnostic_msgs::msg::KeyValue> SensorHandler::fill_details() {
  return {};
}

// | -------------------- support functions ------------------- |

double SensorHandler::calculateRate(std::deque<rclcpp::Time> &timestamps) {
  if (timestamps.size() < 2) {
    return -1.0;
  }

  double dt = (timestamps.back() - timestamps.front()).seconds();
  if (dt <= 0.0) {
    return -1.0;
  }

  return static_cast<double>(timestamps.size() - 1) / dt;
}

uint8_t SensorHandler::mapSensorType(const std::string &type_str) {
  static const std::unordered_map<std::string, uint8_t> type_map = {
      {"Autopilot", mrs_msgs::msg::SensorStatus::TYPE_AUTOPILOT},
      {"Rangefinder", mrs_msgs::msg::SensorStatus::TYPE_RANGEFINDER},
      {"GPS", mrs_msgs::msg::SensorStatus::TYPE_GPS},
      {"IMU", mrs_msgs::msg::SensorStatus::TYPE_IMU},
      {"Barometer", mrs_msgs::msg::SensorStatus::TYPE_BAROMETER},
      {"Magnetometer", mrs_msgs::msg::SensorStatus::TYPE_MAGNETOMETER},
      {"Lidar", mrs_msgs::msg::SensorStatus::TYPE_LIDAR},
      {"Camera", mrs_msgs::msg::SensorStatus::TYPE_CAMERA},
  };

  auto it = type_map.find(type_str);
  if (it != type_map.end()) {
    return it->second;
  }

  RCLCPP_WARN(rclcpp::get_logger("GenericSensorHandler"), "Unknown sensor type '%s', defaulting to TYPE_AUTOPILOT (0)", type_str.c_str());
  return mrs_msgs::msg::SensorStatus::TYPE_AUTOPILOT;
}

Eigen::Matrix3d SensorHandler::cov2eigen(const std::array<double, 9> &msg_cov) {
  Eigen::Matrix3d cov;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      cov(r, c) = msg_cov.at(r + 3 * c);
  return cov;
}

} // namespace mrs_robot_diagnostics
