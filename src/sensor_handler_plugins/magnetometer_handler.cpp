#include <mrs_robot_diagnostics/sensor_plugins/magnetometer_handler.h>

namespace mrs_robot_diagnostics
{
namespace magnetometer_handler
{

bool MagnetometerHandler::onInitialize(rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                                       [[maybe_unused]] const std::string &name_space, [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  RCLCPP_INFO(node->get_logger(), "[MagnetometerHandler] Initializing '%s', topic: '%s'", name_.c_str(), topic_.c_str());

  sh_magnetic_field_ = create_main_subscriber<sensor_msgs::msg::MagneticField>(node, topic_, cbkgrp_subs);
  return true;
}

std::vector<diagnostic_msgs::msg::KeyValue> MagnetometerHandler::fill_details() {

  std::vector<diagnostic_msgs::msg::KeyValue> details;

  auto magnetic_field_msg = sh_magnetic_field_.getMsg();

  if (!magnetic_field_msg) {
    // Initialize with default values if no GPS data has been received yet
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "strength";
    info.value = "nan";
    details.push_back(info);
    info.key   = "uncertainty";
    info.value = "nan";
    details.push_back(info);
  } else {
    diagnostic_msgs::msg::KeyValue info;
    info.key                  = "uncertainty";
    const Eigen::Matrix3d cov = cov2eigen(magnetic_field_msg->magnetic_field_covariance);
    info.value                = std::to_string(std::cbrt(cov.determinant()));
    // Add uncertainty
    details.push_back(info);
    info.key = "strength";
    const Eigen::Vector3d mag(magnetic_field_msg->magnetic_field.x, magnetic_field_msg->magnetic_field.y, magnetic_field_msg->magnetic_field.z);
    info.value = std::to_string(mag.norm());
    // Add strength
    details.push_back(info);
  }

  return details;
}

} // namespace magnetometer_handler
} // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::magnetometer_handler::MagnetometerHandler, mrs_robot_diagnostics::SensorHandler)
