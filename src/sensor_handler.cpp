#include <mrs_robot_diagnostics/sensor_handler.hpp>
#include <unordered_map>

namespace mrs_robot_diagnostics
{

bool SensorHandler::initialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                               rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  mrs_lib::ParamLoader param_loader(node, "SensorHandler");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.setPrefix("robot_diagnostics/sensor_handlers/");

  name_ = config_key; // default name is the config key
  // Load all common parameters using the YAML key (config_key)
  std::string sensor_type_str;
  param_loader.loadParam(config_key + "/topic", topic_);
  param_loader.loadParam(config_key + "/type", sensor_type_str);
  param_loader.loadParam(config_key + "/expected_publisher/node", expected_publisher_node_, std::string("HwApiManager"));
  param_loader.loadParam(config_key + "/expected_publisher/component", expected_publisher_component_, std::string("main"));
  param_loader.loadParam(config_key + "/expected_rate", expected_rate_);
  param_loader.loadParam(config_key + "/rate_tolerance", rate_tolerance_, 0.3);

  std::string qos_reliability;
  param_loader.loadParam(config_key + "/qos_reliability", qos_reliability, std::string("reliable"));

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node->get_logger(), "[SensorHandler] Failed to load config for sensor handler '%s', not initializing", config_key.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "[SensorHandler] Loaded config for '%s' (topic: '%s')", name_.c_str(), topic_.c_str());

  std::string handler_instance = name_ + " handler (" + topic_ + ")";
  error_publisher_             = std::make_shared<mrs_lib::errorgraph::ErrorPublisher>(node, node->get_clock(), "StateMonitor", handler_instance);

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
  msg_count_.store(0, std::memory_order_relaxed);
  rate_tracker_.clear();


  const bool initialized = onInitialize(node, config_key, name_space, cbkgrp_subs);
  is_initialized_        = initialized;

  return initialized;
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
    ss.details = fill_details();
    return ss;
  }

  rclcpp::Time now                = rclcpp::Clock(RCL_STEADY_TIME).now();
  double       elapsed_since_init = (now - init_time_).seconds();

  // Grace period — don't report rate errors right after startup
  if (elapsed_since_init < GRACE_PERIOD_S) {
    ss.ready   = false;
    ss.rate    = rate_tracker_.rate();
    ss.message = "Initializing (grace period)";
    ss.level   = mrs_msgs::msg::SensorStatus::STALE;
    ss.details = fill_details();
    return ss;
  }

  // No messages ever received
  if (msg_count_.load(std::memory_order_relaxed) == 0) {
    ss.ready   = false;
    ss.rate    = 0.0;
    ss.message = "No messages received for " + std::to_string(elapsed_since_init) + " seconds since startup";
    ss.level   = mrs_msgs::msg::SensorStatus::ERROR;
    mrs_lib::errorgraph::node_id_t source_node;
    source_node.node      = expected_publisher_node_;
    source_node.component = expected_publisher_component_;
    error_publisher_->addWaitingForTopicError(topic_, source_node);
    ss.details = fill_details();
    return ss;
  }

  // Topic gone silent — no message for 3x the expected period
  rclcpp::Time last_msg;
  {
    std::scoped_lock lock(mutex_last_msg_);
    last_msg = last_msg_wall_time_;
  }
  double time_since_last = (now - last_msg).seconds();
  double expected_period = 1.0 / expected_rate_;

  if (time_since_last > expected_period * 3.0) {
    ss.ready   = false;
    ss.rate    = 0.0;
    ss.message = "No messages received for " + std::to_string(time_since_last) + " seconds";
    ss.level   = mrs_msgs::msg::SensorStatus::ERROR;
    mrs_lib::errorgraph::node_id_t source_node;
    source_node.node      = expected_publisher_node_;
    source_node.component = expected_publisher_component_;
    error_publisher_->addWaitingForTopicError(topic_, source_node);

    ss.details = fill_details();
    return ss;
  }

  // Rate comparison
  const double measured_rate = rate_tracker_.rate();
  ss.rate                    = measured_rate;

  double lower_bound = expected_rate_ * (1.0 - rate_tolerance_);
  double upper_bound = expected_rate_ * (1.0 + rate_tolerance_);

  if (measured_rate >= lower_bound && measured_rate <= upper_bound) {
    ss.ready   = true;
    ss.level   = mrs_msgs::msg::SensorStatus::OK;
    ss.message = "Rate within expected range";
  } else if (measured_rate < lower_bound) {
    ss.ready   = false;
    ss.level   = mrs_msgs::msg::SensorStatus::WARN;
    ss.message = "Rate too low: expected " + std::to_string(expected_rate_) + " Hz, got " + std::to_string(measured_rate) + " Hz";
  } else {
    ss.ready   = true;
    ss.level   = mrs_msgs::msg::SensorStatus::WARN;
    ss.message = "Rate too high: expected " + std::to_string(expected_rate_);
  }

  ss.details = fill_details();

  return ss;
}

bool SensorHandler::onInitialize([[maybe_unused]] rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                                 [[maybe_unused]] const std::string &name_space, [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {
  return true;
}

std::vector<diagnostic_msgs::msg::KeyValue> SensorHandler::fill_details() {
  return {};
}

// | -------------------- support functions ------------------- |

uint8_t SensorHandler::mapSensorType(const std::string &type_str) {
  static const std::unordered_map<std::string, uint8_t> type_map = {
      {"Autopilot", mrs_msgs::msg::SensorStatus::TYPE_AUTOPILOT},
      {"Rangefinder", mrs_msgs::msg::SensorStatus::TYPE_RANGEFINDER},
      {"GNSS", mrs_msgs::msg::SensorStatus::TYPE_GNSS},
      {"IMU", mrs_msgs::msg::SensorStatus::TYPE_IMU},
      {"Barometer", mrs_msgs::msg::SensorStatus::TYPE_BAROMETER},
      {"Magnetometer", mrs_msgs::msg::SensorStatus::TYPE_MAGNETOMETER},
      {"Lidar", mrs_msgs::msg::SensorStatus::TYPE_LIDAR},
      {"Camera", mrs_msgs::msg::SensorStatus::TYPE_CAMERA},
      {"RemoteController", mrs_msgs::msg::SensorStatus::TYPE_REMOTE_CONTROLLER},
  };

  auto it = type_map.find(type_str);
  if (it != type_map.end()) {
    return it->second;
  }

  RCLCPP_WARN(rclcpp::get_logger("GenericSensorHandler"), "Unknown sensor type '%s'", type_str.c_str());
  return mrs_msgs::msg::SensorStatus::TYPE_UNKNOWN;
}

Eigen::Matrix3d SensorHandler::cov2eigen(const std::array<double, 9> &msg_cov) {
  Eigen::Matrix3d cov;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      cov(r, c) = msg_cov.at(r + 3 * c);
  return cov;
}

} // namespace mrs_robot_diagnostics
