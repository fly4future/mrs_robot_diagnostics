#pragma once

#include <mrs_robot_diagnostics/sensor_handler.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mrs_msgs/msg/gps_info.hpp>

namespace mrs_robot_diagnostics
{
namespace gps_handler
{

class GPSHandler : public mrs_robot_diagnostics::SensorHandler {
public:
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_gnns_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>      sh_gnss_status_;
};

} // namespace gps_handler
} // namespace mrs_robot_diagnostics
