#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/param_loader.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <optional>
#include <cmath>

namespace mrs_robot_diagnostics
{

namespace preflight_checker
{

class PreflightChecker

{
public:
  PreflightChecker(rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs, const std::string &robot_name);

  /** @brief Result of running the full preflight check suite. */
  struct PreflightResult
  {
    bool                     speed_ok       = true;
    bool                     height_ok      = true;
    bool                     gyro_ok        = true;
    bool                     topics_ok      = true;
    bool                     position_valid = true;
    bool                     can_takeoff    = true; ///< AND of all individual checks
    std::vector<std::string> violations;            ///< human-readable failure reasons
  };

  struct PreflightInputs
  {
    std::optional<geometry_msgs::msg::Vector3> velocity;
    std::optional<sensor_msgs::msg::Range>     distance_sensor_range;
    // std::optional<sensor_msgs::msg::Imu>       imu_data;
    std::optional<geometry_msgs::msg::Vector3> angular_rate;
    bool                                       has_distance_sensor = false;
    bool                                       has_imu             = false;
    bool                                       position_valid      = false; // from safety area manager diagnostics
  };

  /** @brief Run speed / height / gyro / topic / position checks; updates debounce timestamps. */
  PreflightResult runPreflightChecks(const PreflightInputs &inputs);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  std::string              robot_name_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_; ///< callback group for subscribers

  void initialize(void);

  /** @brief Individual per-check helpers. */
  std::optional<std::string>              preflightCheckSpeed(const std::optional<geometry_msgs::msg::Vector3> &velocity);
  std::optional<std::string>              preflightCheckHeight(const std::optional<sensor_msgs::msg::Range> &distance_sensor_range, bool has_distance_sensor);
  std::optional<std::string>              preflightCheckGyro(const std::optional<geometry_msgs::msg::Vector3> &angular_rate, bool has_imu);
  std::optional<std::vector<std::string>> preflightCheckTopics(void);

  /** @brief Generic callback that marks a heartbeat topic as seen. */
  void genericTopicCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, size_t id);

  /** @brief Static configuration for the preflight check suite. */
  struct PreflightConfig
  {
    bool   enabled     = false;
    double time_window = 5.0;

    bool   speed_check_enabled = false;
    double speed_check_max     = 0.0;

    bool   height_check_enabled = false;
    double height_check_max     = 0.0;

    bool   gyro_check_enabled = false;
    double gyro_check_max     = 0.0;

    bool                     topic_check_enabled = false;
    double                   topic_check_timeout = 0.0;
    std::vector<std::string> topic_check_topics; // "name:type" entries
  };

  PreflightConfig preflight_cfg_;

  /** @brief Per-check timestamp of the last observed violation (0 = none). */
  rclcpp::Time speed_check_violated_time_;
  rclcpp::Time height_check_violated_time_;
  rclcpp::Time gyro_check_violated_time_;

  /** @brief Tracks last-message time for one topic in the generic topic_check. */
  struct TopicHeartbeat
  {
    std::string  name;
    rclcpp::Time last_msg_time;
  };
  std::mutex                                          topic_heartbeats_mutex_;
  std::vector<TopicHeartbeat>                         topic_heartbeats_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> topic_check_subs_;
};

} // namespace preflight_checker
} // namespace mrs_robot_diagnostics
