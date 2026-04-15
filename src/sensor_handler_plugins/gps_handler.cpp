#include <mrs_robot_diagnostics/sensor_plugins/gps_handler.h>

namespace mrs_robot_diagnostics
{
namespace gps_handler
{

bool GPSHandler::onInitialize(rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key, [[maybe_unused]] const std::string &name_space,
                              [[maybe_unused]] rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  RCLCPP_INFO(node->get_logger(), "[GPSHandler] Initializing '%s', topic: '%s'", name_.c_str(), topic_.c_str());

  // Create subscriber handlers for GPS data and status
  sh_gnns_        = create_main_subscriber<sensor_msgs::msg::NavSatFix>(node, topic_);
  sh_gnss_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>(shopts_, "~/hw_api_gnss_status_in");
  return true;
}

std::vector<diagnostic_msgs::msg::KeyValue> GPSHandler::fill_details() {

  std::vector<diagnostic_msgs::msg::KeyValue> details;

  auto gnss_msg        = sh_gnns_.getMsg();
  auto gnss_status_msg = sh_gnss_status_.getMsg();

  if (!gnss_msg) {
    // Initialize with default values if no GPS data has been received yet
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "uncertainty";
    info.value = "nan"; 
    details.push_back(info);
  } else {
    diagnostic_msgs::msg::KeyValue info;
    info.key                  = "uncertainty";
    const Eigen::Matrix3d cov = cov2eigen(gnss_msg->position_covariance);
    info.value                = std::to_string(std::cbrt(cov.determinant()));
    details.push_back(info);
  }

  if (!gnss_status_msg) {
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "fix_type";
    info.value = "nan"; 
    details.push_back(info);
    info.key   = "num_satellites";
    info.value = "nan"; 
    details.push_back(info);
  } else {
    diagnostic_msgs::msg::KeyValue info;
    info.key   = "fix_type";
    info.value = std::to_string(gnss_status_msg->fix_type);
    details.push_back(info);
    info.key   = "num_satellites";
    info.value = std::to_string(gnss_status_msg->satellites_visible);
    details.push_back(info);
  }

  return details;
}

} // namespace gps_handler
} // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::gps_handler::GPSHandler, mrs_robot_diagnostics::SensorHandler)
