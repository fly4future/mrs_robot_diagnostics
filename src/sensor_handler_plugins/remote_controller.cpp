#include <mrs_robot_diagnostics/sensor_plugins/remote_controller.h>

namespace mrs_robot_diagnostics
{
namespace remote_controller
{

bool RemoteController::onInitialize(rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                                    [[maybe_unused]] const std::string &name_space, [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  RCLCPP_INFO(node->get_logger(), "[RemoteController] Initializing '%s', topic: '%s'", name_.c_str(), topic_.c_str());

  sh_rc_rssi_ = create_main_subscriber<mrs_msgs::msg::HwApiRcRssi>(node, topic_);
  return true;
}

std::vector<diagnostic_msgs::msg::KeyValue> RemoteController::fill_details() {

  std::vector<diagnostic_msgs::msg::KeyValue> details;

  auto rc_rssi_msg = sh_rc_rssi_.getMsg();

  if (!rc_rssi_msg) {
    // Initialize with default values if no GPS data has been received yet
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "rssi";
    info.value = "nan";
    details.push_back(info);
  } else {
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "rssi";
    info.value = std::to_string(rc_rssi_msg->rssi);
    // Add RSSI
    details.push_back(info);
  }

  return details;
}

} // namespace remote_controller
} // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::remote_controller::RemoteController, mrs_robot_diagnostics::SensorHandler)
