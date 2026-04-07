#include <mrs_robot_diagnostics/sensor_plugins/generic_handler.h>

#include <unordered_map>

namespace mrs_robot_diagnostics
{
namespace generic_handler
{

bool GenericSensorHandler::initialize(rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &name_space, const std::string &topic,
                                      rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  name_  = name;
  topic_ = topic;

  mrs_lib::ParamLoader param_loader(node, "GenericSensorHandler");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.setPrefix("robot_diagnostics/sensor_handlers/");

  // Find config block by matching topic
  std::vector<std::string> handler_names;
  param_loader.loadParam("sensor_handler_names", handler_names);

  std::string generic_handler_key;
  for (const auto &key : handler_names) {
    std::string key_topic;
    param_loader.loadParam(key + "/topic", key_topic, std::string(""));
    if (key_topic == topic) {
      generic_handler_key = key;
      break;
    }
  }

  if (generic_handler_key.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[GenericSensorHandler] Could not find config block matching topic '%s'", topic.c_str());
    return false;
  }

  // Read GenericSensorHandler-specific params
  std::string message_type;
  std::string sensor_type_str;
  param_loader.loadParam(generic_handler_key + "/message_type", message_type);
  param_loader.loadParam(generic_handler_key + "/expected_rate", expected_rate_);
  param_loader.loadParam(generic_handler_key + "/rate_tolerance", rate_tolerance_, 0.3);
  param_loader.loadParam(generic_handler_key + "/type", sensor_type_str);

  std::string qos_reliability;
  param_loader.loadParam(generic_handler_key + "/qos_reliability", qos_reliability, std::string("reliable"));

  sensor_type_uint_ = mapSensorType(sensor_type_str);

  // Create QoS profile based on config
  rclcpp::QoS qos_profile(10);
  if (qos_reliability == "best_effort") {
    qos_profile.best_effort();
  } else {
    qos_profile.reliable();
  }
  qos_profile.durability_volatile();

  // Create the generic subscription
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbkgrp_subs;

  generic_sub_ = node->create_generic_subscription(
      topic, message_type, qos_profile, [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) { this->messageCallback(msg); }, sub_options);

  // Initialize timing
  init_time_          = rclcpp::Clock(RCL_STEADY_TIME).now();
  last_msg_wall_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

  RCLCPP_INFO(node->get_logger(), "[GenericSensorHandler] '%s' initialized: topic='%s', msg_type='%s', expected_rate=%.1f Hz, tolerance=%.0f%%", name_.c_str(),
              topic.c_str(), message_type.c_str(), expected_rate_, rate_tolerance_ * 100.0);

  is_initialized_ = true;
  return true;
}

void GenericSensorHandler::messageCallback(const std::shared_ptr<const rclcpp::SerializedMessage> & /*msg*/) {
  std::scoped_lock lock(mutex_timestamps_);

  rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();

  msg_timestamps_.push_back(now);
  if (msg_timestamps_.size() > RATE_WINDOW_SIZE) {
    msg_timestamps_.pop_front();
  }

  msg_count_++;
  last_msg_wall_time_ = now;
  measured_rate_      = calculateWindowedRate();
}

double GenericSensorHandler::calculateWindowedRate() {
  // Called with mutex_timestamps_ already locked
  if (msg_timestamps_.size() < 2) {
    return -1.0;
  }

  double dt = (msg_timestamps_.back() - msg_timestamps_.front()).seconds();
  if (dt <= 0.0) {
    return -1.0;
  }

  return static_cast<double>(msg_timestamps_.size() - 1) / dt;
}

mrs_msgs::msg::SensorStatus GenericSensorHandler::updateStatus() {
  mrs_msgs::msg::SensorStatus ss;
  ss.name  = name_;
  ss.type  = sensor_type_uint_;
  ss.topic = topic_;

  if (!is_initialized_) {
    ss.ready  = false;
    ss.rate   = -1.0;
    ss.status = "NOT_INITIALIZED";
    return ss;
  }

  RCLCPP_INFO(rclcpp::get_logger("GenericSensorHandler"), "[GenericSensorHandler] Updating status for '%s': measured_rate=%.2f Hz, expected_rate=%.2f Hz",
              name_.c_str(), measured_rate_, expected_rate_);

  std::scoped_lock lock(mutex_timestamps_);

  rclcpp::Time now                = rclcpp::Clock(RCL_STEADY_TIME).now();
  double       elapsed_since_init = (now - init_time_).seconds();

  // Grace period — don't report rate errors right after startup
  if (elapsed_since_init < GRACE_PERIOD_S) {
    ss.ready  = false;
    ss.rate   = measured_rate_;
    ss.status = "INITIALIZING";
    return ss;
  }

  // No messages ever received
  if (msg_count_ == 0) {
    ss.ready  = false;
    ss.rate   = 0.0;
    ss.status = "NO_DATA";
    return ss;
  }

  // Topic gone silent — no message for 3x the expected period
  double time_since_last = (now - last_msg_wall_time_).seconds();
  double expected_period = 1.0 / expected_rate_;

  if (time_since_last > expected_period * 3.0) {
    ss.ready  = false;
    ss.rate   = 0.0;
    ss.status = "TIMEOUT";
    return ss;
  }

  // Rate comparison
  ss.rate = measured_rate_;

  double lower_bound = expected_rate_ * (1.0 - rate_tolerance_);
  double upper_bound = expected_rate_ * (1.0 + rate_tolerance_);

  if (measured_rate_ >= lower_bound && measured_rate_ <= upper_bound) {
    ss.ready  = true;
    ss.status = "OK";
  } else if (measured_rate_ < lower_bound) {
    ss.ready  = false;
    ss.status = "RATE_TOO_LOW";
  } else {
    ss.ready  = true;
    ss.status = "RATE_TOO_HIGH";
  }

  return ss;
}

uint8_t GenericSensorHandler::mapSensorType(const std::string &type_str) {
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
  return 0;
}

} // namespace generic_handler
} // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::generic_handler::GenericSensorHandler, mrs_robot_diagnostics::SensorHandler)
