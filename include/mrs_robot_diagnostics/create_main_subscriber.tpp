#pragma once

namespace mrs_robot_diagnostics
{

template <typename MessageType>
mrs_lib::SubscriberHandler<MessageType> SensorHandler::create_main_subscriber(rclcpp::Node::SharedPtr &node, const std::string &topic_name,
                                                                              rclcpp::CallbackGroup::SharedPtr cbkgrp_subs,
                                                                              const rclcpp::Duration          &timeout) {

  shopts_.node                                = node;
  shopts_.node_name                           = "StateMonitor";
  shopts_.no_message_timeout                  = timeout;
  shopts_.threadsafe                          = true;
  shopts_.autostart                           = true;
  shopts_.subscription_options.callback_group = cbkgrp_subs;
  shopts_.qos                                 = qos_profile_;

  auto callback = [this]([[maybe_unused]] const typename MessageType::ConstPtr &msg) {
    rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
    {
      std::scoped_lock lock(mutex_timestamps_);

      msg_timestamps_.push_back(now);
      if (msg_timestamps_.size() > RATE_WINDOW_SIZE) {
        msg_timestamps_.pop_front();
      }

      msg_count_++;
      last_msg_wall_time_ = now;

      std::deque<rclcpp::Time> timestamps_copy = msg_timestamps_;

      measured_rate_ = calculateRate(timestamps_copy);
    }
  };

  return mrs_lib::SubscriberHandler<MessageType>(shopts_, topic_name, callback);
}

} // namespace mrs_robot_diagnostics
