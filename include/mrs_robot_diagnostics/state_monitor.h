#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_element_array.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/msg/uav_diagnostics.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state_estimation_info.hpp>
#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/collision_avoidance_info.hpp>
#include <mrs_msgs/msg/uav_info.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/msg/system_health_info.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mrs_msgs/msg/uav_status.hpp>
#include <mrs_msgs/msg/mpc_tracker_diagnostics.hpp>
#include <mrs_msgs/msg/sensor_status.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/cpu_load.hpp>
#include <std_msgs/msg/float64.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/errorgraph/errorgraph.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/subscriber_handler.h>

#include <mrs_robot_diagnostics/enums/uav_state.h>
#include <mrs_robot_diagnostics/enums/tracker_state.h>
#include <mrs_robot_diagnostics/enums/robot_type.h>
#include <mrs_robot_diagnostics/enums/enum_helpers.h>

#include <mrs_robot_diagnostics/sensor_handler.h>

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

namespace mrs_robot_diagnostics
{
namespace state_monitor
{

class SensorHandlerParams {

public:
  SensorHandlerParams(const std::string &address, const std::string &name_space, const std::string &sensor_name, const std::string &type,
                      const std::string &topic)
      : address(address)
      , name_space(name_space)
      , sensor_name(sensor_name)
      , type(type)
      , topic(topic) {
  }

public:
  std::string address;
  std::string name_space;
  std::string sensor_name;
  std::string type;
  std::string topic;
};

class StateMonitor : public mrs_lib::Node {

public:
  StateMonitor(rclcpp::NodeOptions options);

  template <typename T>
  struct subscriptionResult_t
  {
    bool                       hasNewMessage;
    typename T::ConstSharedPtr message;
  };

private:
  using out_diags_msg_t = mrs_msgs::msg::UavDiagnostics;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  void initialize(void);
  void shutdown();

  std::atomic<bool> is_initialized_ = false;
  std::string       _uav_name_;
  std::string       _body_frame_;

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  std::mutex                          uav_state_mutex_;
  enum_helpers::enum_updater<state_t> uav_state_;

  std::mutex errorgraph_mtx_;

  // TODO to test
  mrs_lib::errorgraph::Errorgraph      errorgraph_;
  rclcpp::Duration                     not_reporting_delay_;
  const mrs_lib::errorgraph::node_id_t autostart_node_id_ = {"AutomaticStart", "main"};

  std::string  _robot_name_;
  std::string  robot_ip_address_;
  robot_type_t robot_type_;

  std::vector<mrs_msgs::msg::SensorStatus> available_sensors_;

  // | ---------------------- ROS subscribers --------------------- |
  std::shared_ptr<mrs_lib::TimeoutManager> tim_mgr_;

  // | -------------------- GeneralRobotInfo -------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::GeneralRobotInfo> ph_general_robot_info_;
  mrs_msgs::msg::GeneralRobotInfo                            last_general_robot_info_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Bool>            sh_automatic_start_can_takeoff_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState> sh_battery_state_;

  // | ------------------- StateEstimationInfo ------------------ |
  mrs_lib::PublisherHandler<mrs_msgs::msg::StateEstimationInfo>    ph_state_estimation_info_;
  mrs_msgs::msg::StateEstimationInfo                               last_state_estimation_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics> sh_estimation_diagnostics_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>          sh_hw_api_gnss_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>        sh_control_manager_heading_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>        sh_hw_api_mag_heading_;
  mrs_lib::SubscriberHandler<std_msgs::msg::UInt8>                  sh_hw_api_rc_rssi_;

  // | ----------------------- ControlInfo ---------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::ControlInfo>                ph_control_info_;
  mrs_msgs::msg::ControlInfo                                           last_control_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diagnostics_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                   sh_control_manager_thrust_;

  // | ----------------- CollisionAvoidanceInfo ----------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::CollisionAvoidanceInfo> ph_collision_avoidance_info_;
  mrs_msgs::msg::CollisionAvoidanceInfo                            last_collision_avoidance_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::MpcTrackerDiagnostics> sh_mpc_tracker_diagnostics_;

  // | ------------------------- UavInfo ------------------------ |
  mrs_lib::PublisherHandler<mrs_msgs::msg::UavInfo>      ph_uav_info_;
  mrs_msgs::msg::UavInfo                                 last_uav_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus> sh_hw_api_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>   sh_uav_status_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>     sh_mass_nominal_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>     sh_mass_estimate_;

  // | -------------------- SystemHealthInfo -------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::SystemHealthInfo>  ph_system_health_info_;
  mrs_msgs::msg::SystemHealthInfo                             last_system_health_info_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField> sh_hw_api_magnetic_field_;

  // | ---------------------- WiFi info ----------------------- |
  struct WifiInfo
  {
    std::string interface;
    float       signal_dbm   = std::numeric_limits<float>::quiet_NaN();
    int32_t     link_quality = -1;
  };

  std::string  _wifi_interface_; // configurable, empty = auto-detect first interface
  WifiInfo     cached_wifi_info_;
  rclcpp::Time last_wifi_read_time_;
  static constexpr double WIFI_READ_INTERVAL_S = 1.0;
  WifiInfo     readWifiInfo();

  // | ------------------------ UAV state ----------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::State> ph_uav_state_;

  // |

  // | ----------------------- Root errors ----------------------- |
  mrs_lib::PublisherHandler<mrs_msgs::msg::ErrorgraphElementArray> ph_root_errors_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ErrorgraphElement>     sh_errorgraph_error_msg_;

  std::unique_ptr<pluginlib::ClassLoader<mrs_robot_diagnostics::SensorHandler>>
                                                                     sensor_handler_loader_;  // pluginlib loader of dynamically loaded sensor handlers
  std::vector<std::string>                                           _sensor_handler_names_;  // list of sensor handlers names
  std::map<std::string, SensorHandlerParams>                         sensor_handlers_params_; // map between sensor handler names and params
  std::vector<std::shared_ptr<mrs_robot_diagnostics::SensorHandler>> sensor_handlers_;        // list of sensor handlers, routines are callable from this
  std::mutex                                                         mutex_sensor_handler_list_;

  // | ----------------------- main timer ----------------------- |

  // timer for main loop
  std::shared_ptr<TimerType> timer_main_;
  void                       timerMain();

  // timer error publishing
  std::shared_ptr<TimerType> timer_error_publishing_;
  void                       timerErrorPublishing();

  // timer for uav state publishing
  std::shared_ptr<TimerType> timer_uav_state_;
  void                       timerUavState();

  // timer for sensor status updating
  std::shared_ptr<TimerType> timer_update_sensor_status_;
  void                       timerUpdateSensorStatus();

  // | ------------------------ Callbacks ----------------------- |
  // TODO to test errorgraph_
  void cbk_errorgraph_element(const mrs_msgs::msg::ErrorgraphElement::ConstSharedPtr element_msg);

  // | ------------------ Additional functions ------------------ |
  std::vector<std::string> extractComponents(const std::string &input);

  template <typename sh_T>
  subscriptionResult_t<sh_T> processIncomingMessage(mrs_lib::SubscriberHandler<sh_T> &sh);

  tracker_state_t parse_tracker_state(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics);
  robot_type_t    parse_robot_type(const std::string &robot_type_str);

  state_t                               parse_uav_state(mrs_msgs::msg::HwApiStatus::ConstSharedPtr               hw_api_status,
                                                        mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics);
  mrs_msgs::msg::GeneralRobotInfo       parse_general_robot_info(sensor_msgs::msg::BatteryState::ConstSharedPtr battery_state);
  mrs_msgs::msg::StateEstimationInfo    parse_state_estimation_info(mrs_msgs::msg::EstimationDiagnostics::ConstSharedPtr estimation_diagnostics,
                                                                    mrs_msgs::msg::Float64Stamped::ConstSharedPtr        local_heading,
                                                                    sensor_msgs::msg::NavSatFix::ConstSharedPtr          global_position,
                                                                    mrs_msgs::msg::Float64Stamped::ConstSharedPtr        global_heading);
  mrs_msgs::msg::ControlInfo            parse_control_info(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics,
                                                           std_msgs::msg::Float64::ConstSharedPtr                   thrust);
  mrs_msgs::msg::CollisionAvoidanceInfo parse_collision_avoidance_info(mrs_msgs::msg::MpcTrackerDiagnostics::ConstSharedPtr mpc_tracker_diagnostics);
  mrs_msgs::msg::UavInfo          parse_uav_info(mrs_msgs::msg::HwApiStatus::ConstSharedPtr hw_api_status, mrs_msgs::msg::UavStatus::ConstSharedPtr uav_status,
                                                 std_msgs::msg::Float64::ConstSharedPtr mass_nominal, std_msgs::msg::Float64::ConstSharedPtr mass_estimate);
  mrs_msgs::msg::SystemHealthInfo parse_system_health_info(mrs_msgs::msg::UavStatus::ConstSharedPtr        uav_status,
                                                           sensor_msgs::msg::NavSatFix::ConstSharedPtr     gnss,
                                                           sensor_msgs::msg::MagneticField::ConstSharedPtr magnetic_field,
                                                           std_msgs::msg::UInt8::ConstSharedPtr  rc_rssi);

  mrs_msgs::msg::GeneralRobotInfo       init_general_robot_info();
  mrs_msgs::msg::StateEstimationInfo    init_state_estimation_info();
  mrs_msgs::msg::ControlInfo            init_control_info();
  mrs_msgs::msg::CollisionAvoidanceInfo init_collision_avoidance_info();
  mrs_msgs::msg::UavInfo                init_uav_info();
  mrs_msgs::msg::SystemHealthInfo       init_system_health_info();
};

// Template method definition — must be in the header
template <typename sh_T>
StateMonitor::subscriptionResult_t<sh_T> StateMonitor::processIncomingMessage(mrs_lib::SubscriberHandler<sh_T> &sh) {
  StateMonitor::subscriptionResult_t<sh_T> msg;
  msg.hasNewMessage = sh.newMsg();
  msg.message       = msg.hasNewMessage ? sh.getMsg() : sh.peekMsg();
  if (msg.message != nullptr) {
    if (clock_->now() - sh.lastMsgTime() > not_reporting_delay_) {
      msg.message       = nullptr;
      msg.hasNewMessage = true;
    }
  }
  return msg;
}

} // namespace state_monitor
} // namespace mrs_robot_diagnostics
