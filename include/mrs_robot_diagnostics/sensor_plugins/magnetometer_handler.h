#pragma once

#include <mrs_robot_diagnostics/sensor_handler.h>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace mrs_robot_diagnostics
{
namespace magnetometer_handler
{

class MagnetometerHandler : public mrs_robot_diagnostics::SensorHandler {
public:
  bool onInitialize(rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &name_space, const std::string &topic,
                    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;

  std::vector<diagnostic_msgs::msg::KeyValue> fill_details() override;

private:
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_magnetic_field_;
};

} // namespace magnetometer_handler
} // namespace mrs_robot_diagnostics
