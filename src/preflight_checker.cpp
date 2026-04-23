#include <mrs_robot_diagnostics/preflight_checker.h>

namespace mrs_robot_diagnostics
{
namespace preflight_checker
{

PreflightChecker::PreflightChecker(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs, const std::string &robot_name)
    : node_(node)
    , clock_(node_->get_clock())
    , robot_name_(robot_name)
    , cbkgrp_subs_(cbkgrp_subs) {

  initialize();
}

void PreflightChecker::initialize(void) {
  // Load parameters, set up subscribers, etc.
  mrs_lib::ParamLoader param_loader(node_, "PreflightChecker");
  std::string          custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("preflight_check_config");

  // preflight check configuration
  param_loader.loadParam("robot_diagnostics/preflight_check/enabled", preflight_cfg_.enabled, false);
  param_loader.loadParam("robot_diagnostics/preflight_check/time_window", preflight_cfg_.time_window, 5.0);

  param_loader.loadParam("robot_diagnostics/preflight_check/speed_check/enabled", preflight_cfg_.speed_check_enabled, false);
  param_loader.loadParam("robot_diagnostics/preflight_check/speed_check/max_speed", preflight_cfg_.speed_check_max, 0.0);

  param_loader.loadParam("robot_diagnostics/preflight_check/height_check/enabled", preflight_cfg_.height_check_enabled, false);
  param_loader.loadParam("robot_diagnostics/preflight_check/height_check/max_height", preflight_cfg_.height_check_max, 0.0);

  param_loader.loadParam("robot_diagnostics/preflight_check/gyro_check/enabled", preflight_cfg_.gyro_check_enabled, false);
  param_loader.loadParam("robot_diagnostics/preflight_check/gyro_check/max_rate", preflight_cfg_.gyro_check_max, 0.0);

  param_loader.loadParam("robot_diagnostics/preflight_check/topic_check/enabled", preflight_cfg_.topic_check_enabled, false);
  param_loader.loadParam("robot_diagnostics/preflight_check/topic_check/timeout", preflight_cfg_.topic_check_timeout, 5.0);
  param_loader.loadParam("robot_diagnostics/preflight_check/topic_check/topics", preflight_cfg_.topic_check_topics, std::vector<std::string>{});


  speed_check_violated_time_  = rclcpp::Time(0, 0, clock_->get_clock_type());
  height_check_violated_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  gyro_check_violated_time_   = rclcpp::Time(0, 0, clock_->get_clock_type());

  if (preflight_cfg_.enabled && preflight_cfg_.topic_check_enabled) {
    topic_heartbeats_.reserve(preflight_cfg_.topic_check_topics.size());
    topic_check_subs_.reserve(preflight_cfg_.topic_check_topics.size());

    for (size_t i = 0; i < preflight_cfg_.topic_check_topics.size(); ++i) {
      const std::string &entry = preflight_cfg_.topic_check_topics.at(i);

      // entries are "name:type"
      const auto colon = entry.find(':');
      if (colon == std::string::npos || colon == 0 || colon == entry.size() - 1) {
        RCLCPP_WARN(node_->get_logger(), "preflight topic_check: malformed entry '%s' (expected 'name:type'), skipping", entry.c_str());
        continue;
      }

      std::string topic_name = entry.substr(0, colon);
      std::string topic_type = entry.substr(colon + 1);

      if (topic_name.empty()) {
        continue;
      }
      if (topic_name.front() != '/') {
        topic_name = "/" + robot_name_ + "/" + topic_name;
      }

      TopicHeartbeat hb;
      hb.name          = topic_name;
      hb.last_msg_time = rclcpp::Time(0, 0, clock_->get_clock_type());
      topic_heartbeats_.push_back(hb);

      const size_t id = topic_heartbeats_.size() - 1;

      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cbkgrp_subs_;

      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> cb = [this, id](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        this->genericTopicCallback(msg, id);
      };

      auto sub = node_->create_generic_subscription(topic_name, topic_type, rclcpp::SystemDefaultsQoS(), cb, sub_opts);
      topic_check_subs_.push_back(sub);
    }
  }
}

PreflightChecker::PreflightResult PreflightChecker::runPreflightChecks(const PreflightInputs &inputs) {
  PreflightResult result;

  if (!preflight_cfg_.enabled)
    return result;

  if (auto speed_check_result = preflightCheckSpeed(inputs.velocity)) {
    result.speed_ok = false;
    result.violations.push_back(*speed_check_result);
  }

  if (auto height_check_result = preflightCheckHeight(inputs.distance_sensor_range, inputs.has_distance_sensor)) {
    result.height_ok = false;
    result.violations.push_back(*height_check_result);
  }

  if (auto gyro_check_result = preflightCheckGyro(inputs.angular_rate, inputs.has_imu)) {
    result.gyro_ok = false;
    result.violations.push_back(*gyro_check_result);
  }

  if (auto topic_check_result = preflightCheckTopics()) {
    result.topics_ok          = false;
    auto &topic_check_results = *topic_check_result;
    result.violations.reserve(result.violations.size() + topic_check_results.size());
    result.violations.insert(result.violations.end(), topic_check_results.begin(), topic_check_results.end());
  }

  if (!inputs.position_valid) {
    result.violations.push_back("preflight position: invalid position");
  }

  result.position_valid = inputs.position_valid;

  result.can_takeoff = result.speed_ok && result.height_ok && result.gyro_ok && result.topics_ok && result.position_valid;
  return result;
}

std::optional<std::string> PreflightChecker::preflightCheckSpeed(const std::optional<geometry_msgs::msg::Vector3> &velocity) {
  if (!preflight_cfg_.speed_check_enabled)
    return std::nullopt;

  std::string violation;

  if (!velocity.has_value()) {
    violation = "preflight speed: no velocity received"; 
    return violation;
  }

  const auto  &vel   = velocity.value();
  const double speed = std::hypot(vel.x, vel.y, vel.z);

  if (std::isnan(speed) || speed > preflight_cfg_.speed_check_max) {
    speed_check_violated_time_ = clock_->now();
    std::stringstream ss;
    ss << "preflight speed: " << speed << " m/s exceeds limit " << preflight_cfg_.speed_check_max << " m/s";
    violation = ss.str();
  }

  if (speed_check_violated_time_.seconds() > 0 && (clock_->now() - speed_check_violated_time_).seconds() < preflight_cfg_.time_window) {
    if (violation.empty())
      violation = "preflight speed: still within debounce window after last violation";
    return violation;
  }
  return std::nullopt;
}

std::optional<std::string> PreflightChecker::preflightCheckHeight(const std::optional<sensor_msgs::msg::Range> &distance_sensor_range,
                                                                  bool                                          has_distance_sensor) {
  if (!preflight_cfg_.height_check_enabled || !has_distance_sensor)
    return std::nullopt;

  std::string violation;
  if (!distance_sensor_range.has_value()) {
    violation = "preflight height: no distance sensor range received";
    return violation;
  }

  // const double height = sh_hw_api_distance_sensor_.getMsg()->range;
  const double height = distance_sensor_range->range;

  if (std::isnan(height) || height > preflight_cfg_.height_check_max) {
    height_check_violated_time_ = clock_->now();
    std::stringstream ss;
    ss << "preflight height: " << height << " m exceeds limit " << preflight_cfg_.height_check_max << " m";
    violation = ss.str();
  }

  if (height_check_violated_time_.seconds() > 0 && (clock_->now() - height_check_violated_time_).seconds() < preflight_cfg_.time_window) {
    if (violation.empty())
      violation = "preflight height: still within debounce window after last violation";
    return violation;
  }
  return std::nullopt;
}

std::optional<std::string> PreflightChecker::preflightCheckGyro(const std::optional<geometry_msgs::msg::Vector3> &angular_rate, bool has_imu) {
  if (!preflight_cfg_.gyro_check_enabled || !has_imu)
    return std::nullopt;

  std::string violation;
  if (!angular_rate.has_value()) {
    violation = "preflight gyro: no angular rate received";
    return violation;
  }

  const auto   g   = angular_rate.value();
  const double max = preflight_cfg_.gyro_check_max;

  if (std::isnan(g.x) || std::isnan(g.y) || std::isnan(g.z) || std::abs(g.x) > max || std::abs(g.y) > max || std::abs(g.z) > max) {
    gyro_check_violated_time_ = clock_->now();
    std::stringstream ss;
    ss << "preflight gyro: angular velocity [" << g.x << ", " << g.y << ", " << g.z << "] rad/s exceeds limit " << max << " rad/s";
    violation = ss.str();
  }

  if (gyro_check_violated_time_.seconds() > 0 && (clock_->now() - gyro_check_violated_time_).seconds() < preflight_cfg_.time_window) {
    if (violation.empty())
      violation = "preflight gyro: still within debounce window after last violation";
    return violation;
  }
  return std::nullopt;
}

std::optional<std::vector<std::string>> PreflightChecker::preflightCheckTopics() {
  std::scoped_lock lck(topic_heartbeats_mutex_);
  if (!preflight_cfg_.topic_check_enabled)
    return std::nullopt;

  bool                     all_ok = true;
  const auto               now    = clock_->now();
  std::vector<std::string> violations;

  for (const auto &hb : topic_heartbeats_) {
    const bool never_seen = hb.last_msg_time.seconds() == 0;
    const bool stale      = !never_seen && (now - hb.last_msg_time).seconds() > preflight_cfg_.topic_check_timeout;
    if (never_seen || stale) {
      violations.push_back("preflight topic_check: no recent data on " + hb.name);
      all_ok = false;
    }
  }

  if (all_ok) {
    return std::nullopt;
  } else {
    return violations;
  }
}

void PreflightChecker::genericTopicCallback([[maybe_unused]] const std::shared_ptr<rclcpp::SerializedMessage> msg, size_t id) {
  std::scoped_lock lck(topic_heartbeats_mutex_);
  if (id >= topic_heartbeats_.size())
    return;
  topic_heartbeats_.at(id).last_msg_time = clock_->now();
}
} // namespace preflight_checker
} // namespace mrs_robot_diagnostics
