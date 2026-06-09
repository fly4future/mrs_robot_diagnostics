#include <mrs_robot_diagnostics/sensor_plugins/generic_handler.hpp>

namespace mrs_robot_diagnostics
{
namespace generic_handler
{

bool GenericSensorHandler::onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, [[maybe_unused]] const std::string &name_space,
                                        rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  mrs_lib::ParamLoader param_loader(node, "GenericSensorHandler");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (!custom_config_path.empty()) {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.setPrefix("robot_diagnostics/sensor_handlers/");

  // Read GenericSensorHandler-specific params
  std::string message_type;
  param_loader.loadParam(config_key + "/message_type", message_type);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node->get_logger(), "[GenericSensorHandler] Failed to load config for '%s', not initializing", config_key.c_str());
    error_publisher_->addOneshotError("Failed to load config for generic sensor handler " + name_);
    return false;
  }

  // Create the generic subscription
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbkgrp_subs;

  generic_sub_ = node->create_generic_subscription(
      topic_, message_type, qos_profile_, [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) { this->messageCallback(msg); }, sub_options);

  RCLCPP_INFO(node->get_logger(), "[GenericSensorHandler] '%s' initialized: topic='%s', msg_type='%s', expected_rate=%.1f Hz, tolerance=%.0f%%", name_.c_str(),
              topic_.c_str(), message_type.c_str(), expected_rate_, rate_tolerance_ * 100.0);

  return true;
}

void GenericSensorHandler::messageCallback([[maybe_unused]] const std::shared_ptr<const rclcpp::SerializedMessage> &msg) {
  rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
  rate_tracker_.record(now);
  msg_count_.fetch_add(1, std::memory_order_relaxed);
  {
    std::scoped_lock lock(mutex_last_msg_);
    last_msg_wall_time_ = now;
  }
}

} // namespace generic_handler
} // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::generic_handler::GenericSensorHandler, mrs_robot_diagnostics::SensorHandler)
