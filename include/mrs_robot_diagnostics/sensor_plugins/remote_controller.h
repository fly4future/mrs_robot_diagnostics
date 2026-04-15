#pragma once

#include <mrs_robot_diagnostics/sensor_handler.h>
#include <mrs_msgs/msg/hw_api_rc_rssi.hpp>

namespace mrs_robot_diagnostics
{
namespace remote_controller
{

class RemoteController : public mrs_robot_diagnostics::SensorHandler {
public:
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &name_space, const std::string &topic,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiRcRssi> sh_rc_rssi_;
};

} // namespace remote_controller
} // namespace mrs_robot_diagnostics
