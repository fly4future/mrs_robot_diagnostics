#include <mrs_robot_diagnostics/state_monitor.h>

#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace mrs_robot_diagnostics
{
namespace state_monitor
{

StateMonitor::StateMonitor(rclcpp::NodeOptions options)
    : mrs_lib::Node("state_monitor", options)
    , uav_state_(this_node_ptr()->get_logger(), "UAV STATE", state_t::UNKNOWN)
    , errorgraph_(this_node_ptr()->get_clock())
    , not_reporting_delay_(rclcpp::Duration::from_seconds(0.0)) {


  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  initialize();
}

void StateMonitor::initialize() {
  RCLCPP_INFO(node_->get_logger(), "Initializing...");


  /* load parameters */
  mrs_lib::ParamLoader param_loader(node_, "StateMonitor");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  std::string robot_type;
  param_loader.loadParam("robot_name", _robot_name_);
  param_loader.loadParam("robot_type", robot_type);

  robot_type_ = parse_robot_type(robot_type);

  std::vector<char> hostname(1024);

  if (gethostname(hostname.data(), hostname.size()) == 0) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Hostname: " << hostname.data());
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get hostname");
  }

  if (hostname.data() != _robot_name_) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Hostname '"
                                                << hostname.data() << "' does not match the robot name '" << _robot_name_
                                                << "'. This might lead to issues in IP resolution, if you are using the hostname to connect to the robot, "
                                                   "please check your network configuration and make sure the hostname is correct");
  }

  addrinfo hints{};
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo *res = nullptr;

  if (getaddrinfo(hostname.data(), nullptr, &hints, &res) != 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to resolve "
                                                << hostname.data()
                                                << ", skipping IP resolution for this robot, if you are using the hostname to connect to the robot, "
                                                   "please check your network configuration and make sure the hostname is correct");
  } else {
    char  ip[INET_ADDRSTRLEN];
    void *addr = &((sockaddr_in *)res->ai_addr)->sin_addr;
    inet_ntop(AF_INET, addr, ip, sizeof(ip));

    robot_ip_address_ = std::string(ip);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Resolved IP address: " << robot_ip_address_);

    freeaddrinfo(res);
  }

  param_loader.loadParam("robot_diagnostics/wifi_interface", _wifi_interface_, std::string(""));

  auto       main_timer_rate      = param_loader.loadParam2<double>("robot_diagnostics/main_timer_rate");
  auto       error_publisher_rate = param_loader.loadParam2<double>("robot_diagnostics/error_publisher_rate");
  const auto state_timer_rate     = param_loader.loadParam2<double>("robot_diagnostics/state_timer_rate");
  not_reporting_delay_            = param_loader.loadParam2<rclcpp::Duration>("robot_diagnostics/not_reporting_delay");

  std::string available_sensors_string;
  param_loader.loadParam("available_sensors", available_sensors_string);
  param_loader.setPrefix("robot_diagnostics/sensor_handlers/");
  const auto update_status_rate = param_loader.loadParam2<double>("update_timer_rate");

  param_loader.loadParam("sensor_handler_names", _sensor_handler_names_);

  sensor_handler_loader_ =
      std::make_unique<pluginlib::ClassLoader<mrs_robot_diagnostics::SensorHandler>>("mrs_robot_diagnostics", "mrs_robot_diagnostics::SensorHandler");

  // for each plugin in the list
  for (int i = 0; i < int(_sensor_handler_names_.size()); i++) {
    std::string sensor_handler_name = _sensor_handler_names_[i];

    // load the plugin parameters
    std::string address;
    std::string name_space;
    std::string sensor_name;
    std::string type;
    std::string topic;

    param_loader.loadParam(sensor_handler_name + "/address", address);
    param_loader.loadParam(sensor_handler_name + "/name", sensor_name);
    param_loader.loadParam(sensor_handler_name + "/type", type);
    param_loader.loadParam(sensor_handler_name + "/topic", topic);

    SensorHandlerParams new_sensor_handler(address, _robot_name_, sensor_name, type, topic);
    sensor_handlers_params_.insert(std::pair<std::string, SensorHandlerParams>(sensor_handler_name, new_sensor_handler));

    try {
      RCLCPP_INFO(node_->get_logger(), "loading the sensor handler '%s'", new_sensor_handler.address.c_str());
      sensor_handlers_.push_back(sensor_handler_loader_->createSharedInstance(new_sensor_handler.address.c_str()));
    }
    catch (pluginlib::CreateClassException &ex1) {
      RCLCPP_ERROR(node_->get_logger(), "CreateClassException for the sensor handler '%s'", new_sensor_handler.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex1.what());
      rclcpp::shutdown();
    }
    catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "PluginlibException for the sensor handler '%s'", new_sensor_handler.address.c_str());
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", ex.what());
      rclcpp::shutdown();
    }
  }

  RCLCPP_INFO(node_->get_logger(), "sensor handlers were loaded");
  {
    for (int i = 0; i < int(sensor_handlers_.size()); i++) {
      try {
        std::map<std::string, SensorHandlerParams>::iterator it;
        it = sensor_handlers_params_.find(_sensor_handler_names_[i]);

        RCLCPP_INFO(node_->get_logger(), "initializing the sensor handler'%s'", it->second.address.c_str());
        sensor_handlers_[i]->initialize(node_, it->second.sensor_name, it->second.name_space, it->second.topic, cbkgrp_subs_);
      }
      catch (std::runtime_error &ex) {
        RCLCPP_ERROR(node_->get_logger(), "exception caught during sensor handler initialization '%s'", ex.what());
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Sensor handlers were initialized");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  mrs_msgs::msg::SensorStatus ss_msg;
  ss_msg.ready  = true;
  ss_msg.rate   = -1;
  // ss_msg.status = "NOT_IMPLEMENTED";

  std::vector<std::string> components = extractComponents(available_sensors_string);
  RCLCPP_INFO(node_->get_logger(), "components size: %zu", components.size());
  for (const auto &comp : components) {
    ss_msg.name = comp;
    available_sensors_.push_back(ss_msg);
  }


  // | ----------------------- subscribers ---------------------- |

  tim_mgr_ = std::make_shared<mrs_lib::TimeoutManager>(node_, rclcpp::Rate(1.0));
  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = "StateMonitor";
  shopts.no_message_timeout                  = rclcpp::Duration(not_reporting_delay_);
  shopts.timeout_manager                     = tim_mgr_;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  ph_root_errors_          = mrs_lib::PublisherHandler<mrs_msgs::msg::ErrorgraphElementArray>(node_, "~/root_errors_out");
  sh_errorgraph_error_msg_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ErrorgraphElement>(shopts, "~/errors_in", &StateMonitor::cbk_errorgraph_element, this);

  // | -------------------- GeneralRobotInfo -------------------- |
  ph_general_robot_info_          = mrs_lib::PublisherHandler<mrs_msgs::msg::GeneralRobotInfo>(node_, "~/general_robot_info_out");
  sh_battery_state_               = mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>(shopts, "~/battery_state_in");
  sh_automatic_start_can_takeoff_ = mrs_lib::SubscriberHandler<std_msgs::msg::Bool>(shopts, "~/automatic_start_can_takeoff_in", mrs_lib::no_timeout);

  // | ------------------- StateEstimationInfo ------------------ |
  ph_state_estimation_info_   = mrs_lib::PublisherHandler<mrs_msgs::msg::StateEstimationInfo>(node_, "~/state_estimation_info_out");
  last_state_estimation_info_ = init_state_estimation_info();
  sh_estimation_diagnostics_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diagnostics_in");
  sh_hw_api_gnss_             = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(shopts, "~/hw_api_gnss_in");
  sh_hw_api_gnss_status_      = mrs_lib::SubscriberHandler<mrs_msgs::msg::GpsInfo>(shopts, "~/hw_api_gnss_status_in");
  sh_control_manager_heading_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/control_manager_heading_in");
  sh_hw_api_mag_heading_      = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(shopts, "~/hw_api_mag_heading_in");
  sh_hw_api_rc_rssi_          = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiRcRssi>(shopts, "~/hw_api_rc_rssi_in");

  // | ----------------------- ControlInfo ---------------------- |
  ph_control_info_                   = mrs_lib::PublisherHandler<mrs_msgs::msg::ControlInfo>(node_, "~/control_info_out");
  sh_constraint_manager_diagnostics_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics>(shopts, "~/constraint_manager_diagnostics_in");
  sh_control_manager_diagnostics_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");
  sh_control_manager_thrust_         = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/control_manager_thrust_in");
  sh_gain_manager_diagnostics_       = mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>(shopts, "~/gain_manager_diagnostics_in");

  // | ----------------- CollisionAvoidanceInfo ----------------- |
  ph_collision_avoidance_info_ = mrs_lib::PublisherHandler<mrs_msgs::msg::CollisionAvoidanceInfo>(node_, "~/collision_avoidance_info_out");
  sh_mpc_tracker_diagnostics_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::MpcTrackerDiagnostics>(shopts, "~/mpc_tracker_diagnostics_in");

  // | ------------------------- UavInfo ------------------------ |
  ph_uav_info_      = mrs_lib::PublisherHandler<mrs_msgs::msg::UavInfo>(node_, "~/uav_info_out");
  sh_hw_api_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, "~/hw_api_status_in");
  sh_uav_status_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavStatus>(shopts, "~/uav_status_in");
  sh_mass_nominal_  = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mass_nominal_in");
  sh_mass_estimate_ = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mass_estimate_in");

  // | -------------------- SystemHealthInfo -------------------- |
  ph_system_health_info_    = mrs_lib::PublisherHandler<mrs_msgs::msg::SystemHealthInfo>(node_, "~/system_health_info_out");
  last_wifi_read_time_      = clock_->now();
  sh_hw_api_magnetic_field_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(shopts, "~/hw_api_magnetic_field_in", mrs_lib::no_timeout);

  // | ------------------------ UAV state ----------------------- |
  ph_uav_state_ = mrs_lib::PublisherHandler<mrs_msgs::msg::State>(node_, "~/uav_state_out");

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_timers_;

  {
    std::function<void()> callback_fcn = std::bind(&StateMonitor::timerMain, this);

    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(main_timer_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&StateMonitor::timerErrorPublishing, this);

    timer_error_publishing_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(error_publisher_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&StateMonitor::timerUpdateSensorStatus, this);

    timer_update_sensor_status_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(update_status_rate, clock_), callback_fcn);
  }

  {
    std::function<void()> callback_fcn = std::bind(&StateMonitor::timerUavState, this);

    timer_uav_state_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(state_timer_rate, clock_), callback_fcn);
  }

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(node_->get_logger(), " initialized ");
  RCLCPP_INFO(node_->get_logger(), "--------------------");
  is_initialized_ = true;
}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

void StateMonitor::timerMain() {
  if (!is_initialized_) {
    return;
  }
  std::scoped_lock lck(uav_state_mutex_);
  const auto       now                             = clock_->now();
  const auto       battery_state                   = processIncomingMessage(sh_battery_state_);
  const auto       control_manager_diagnostics     = processIncomingMessage(sh_control_manager_diagnostics_);
  const auto       control_manager_heading         = processIncomingMessage(sh_control_manager_heading_);
  const auto       control_manager_thrust          = processIncomingMessage(sh_control_manager_thrust_);
  const auto       constraint_manager_diagnostics = processIncomingMessage(sh_constraint_manager_diagnostics_);
  const auto       gain_manager_diagnostics        = processIncomingMessage(sh_gain_manager_diagnostics_);
  const auto       estimation_diagnostics          = processIncomingMessage(sh_estimation_diagnostics_);
  const auto       hw_api_gnss                     = processIncomingMessage(sh_hw_api_gnss_);
  const auto       hw_api_gnss_status              = processIncomingMessage(sh_hw_api_gnss_status_);
  const auto       hw_api_mag_heading              = processIncomingMessage(sh_hw_api_mag_heading_);
  const auto       hw_api_magnetic_field           = processIncomingMessage(sh_hw_api_magnetic_field_);
  const auto       hw_api_rc_rssi                  = processIncomingMessage(sh_hw_api_rc_rssi_);
  const auto       hw_api_status                   = processIncomingMessage(sh_hw_api_status_);
  const auto       mass_estimate                   = processIncomingMessage(sh_mass_estimate_);
  const auto       mass_nominal                    = processIncomingMessage(sh_mass_nominal_);
  const auto       mpc_tracker_diagnostics         = processIncomingMessage(sh_mpc_tracker_diagnostics_);
  const auto       uav_status                      = processIncomingMessage(sh_uav_status_);

  if (hw_api_status.hasNewMessage || control_manager_diagnostics.hasNewMessage) {
    const auto new_state = parse_uav_state(hw_api_status.message, control_manager_diagnostics.message);
    uav_state_.set(new_state);
  }

  last_general_robot_info_ = parse_general_robot_info(battery_state.message);

  if (estimation_diagnostics.hasNewMessage || control_manager_heading.hasNewMessage || hw_api_gnss.hasNewMessage || hw_api_mag_heading.hasNewMessage)
    last_state_estimation_info_ =
        parse_state_estimation_info(estimation_diagnostics.message, control_manager_heading.message, hw_api_gnss.message, hw_api_mag_heading.message);

  if (control_manager_diagnostics.hasNewMessage || control_manager_thrust.hasNewMessage || constraint_manager_diagnostics.hasNewMessage ||
      gain_manager_diagnostics.hasNewMessage)
    last_control_info_ = parse_control_info(control_manager_diagnostics.message, constraint_manager_diagnostics.message, gain_manager_diagnostics.message,
                                            control_manager_thrust.message);

  if (mpc_tracker_diagnostics.hasNewMessage)
    last_collision_avoidance_info_ = parse_collision_avoidance_info(mpc_tracker_diagnostics.message);

  if (hw_api_status.hasNewMessage || uav_status.hasNewMessage || mass_nominal.hasNewMessage || mass_estimate.hasNewMessage)
    last_uav_info_ = parse_uav_info(hw_api_status.message, uav_status.message, mass_nominal.message, mass_estimate.message);

  if (uav_status.hasNewMessage || hw_api_gnss.hasNewMessage || hw_api_magnetic_field.hasNewMessage || hw_api_rc_rssi.hasNewMessage ||
      hw_api_gnss_status.hasNewMessage)
    last_system_health_info_ =
        parse_system_health_info(uav_status.message, hw_api_magnetic_field.message, hw_api_rc_rssi.message);

  ph_general_robot_info_.publish(last_general_robot_info_);
  ph_state_estimation_info_.publish(last_state_estimation_info_);
  ph_control_info_.publish(last_control_info_);
  ph_collision_avoidance_info_.publish(last_collision_avoidance_info_);
  ph_uav_info_.publish(last_uav_info_);
  ph_system_health_info_.publish(last_system_health_info_);

  mrs_msgs::msg::State uav_state_msg;
  uav_state_msg.stamp = now;
  uav_state_msg.state = to_ros(uav_state_.value());
  ph_uav_state_.publish(uav_state_msg);


  // to avoid getting timeout warnings on this latched message
  if (sh_mass_nominal_.hasMsg())
    sh_mass_nominal_.setNoMessageTimeout(mrs_lib::no_timeout);
}

void StateMonitor::timerErrorPublishing() {
  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lck(errorgraph_mtx_);

  mrs_msgs::msg::ErrorgraphElementArray root_errors_msg;
  root_errors_msg.stamp = clock_->now();

  const auto root_errors = errorgraph_.find_error_roots();

  for (const auto &error : root_errors) {
    root_errors_msg.elements.push_back(std::visit([](const auto &info) { return info.to_msg(); }, error));
  }

  ph_root_errors_.publish(root_errors_msg);
}

void StateMonitor::timerUavState() {
  if (!is_initialized_) {
    return;
  }
  std::scoped_lock lck(uav_state_mutex_);
  const auto       now                         = clock_->now();
  const auto       hw_api_status               = processIncomingMessage(sh_hw_api_status_);
  const auto       control_manager_diagnostics = processIncomingMessage(sh_control_manager_diagnostics_);

  if (!hw_api_status.hasNewMessage && !control_manager_diagnostics.hasNewMessage)
    return;

  const auto new_state = parse_uav_state(hw_api_status.message, control_manager_diagnostics.message);

  if (new_state == uav_state_.value())
    return;

  uav_state_.set(new_state);

  mrs_msgs::msg::State uav_state_msg;
  uav_state_msg.stamp = now;
  uav_state_msg.state = to_ros(uav_state_.value());
  ph_uav_state_.publish(uav_state_msg);
}

void StateMonitor::timerUpdateSensorStatus() {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lck(mutex_sensor_handler_list_);
  available_sensors_.clear();
  for (auto &handler : sensor_handlers_) {
    auto sensor_status_msg = handler->updateStatus();
    available_sensors_.push_back(sensor_status_msg);
  }
}

// | ------------------------ callbacks ----------------------- |

void StateMonitor::cbk_errorgraph_element(const mrs_msgs::msg::ErrorgraphElement::ConstSharedPtr element_msg) {
  std::scoped_lock lck(errorgraph_mtx_);
  errorgraph_.add_element_from_msg(*element_msg);
}

// | -------------------- support functions ------------------- |

Eigen::Matrix3d cov2eigen(const std::array<double, 9> &msg_cov) {
  Eigen::Matrix3d cov;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      cov(r, c) = msg_cov.at(r + 3 * c);
  return cov;
}

std::vector<std::string> StateMonitor::extractComponents(const std::string &input) {
  std::vector<std::string> result;
  std::stringstream        ss(input);
  std::string              item;

  // stream extraction operator automatically skips delimiters
  while (ss >> item) {
    result.push_back(item);
  }
  return result;
}

robot_type_t StateMonitor::parse_robot_type(const std::string &robot_type_str) {

  // Convert to lowercase for case-insensitive comparison
  std::string lower_str = robot_type_str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), [](unsigned char c) { return std::tolower(c); });

  if (lower_str == "multirotor") {
    return robot_type_t::MULTIROTOR;
  } else if (lower_str == "boat") {
    return robot_type_t::BOAT;
  } else {
    return robot_type_t::UNKNOWN;
  }
}

// | --------------------- Parsing methods -------------------- |

tracker_state_t StateMonitor::parse_tracker_state(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics) {

  if (control_manager_diagnostics == nullptr)
    return tracker_state_t::UNKNOWN;

  if (control_manager_diagnostics->active_tracker == "NullTracker")
    return tracker_state_t::INVALID;

  switch (control_manager_diagnostics->tracker_status.state) {

    case mrs_msgs::msg::TrackerStatus::STATE_INVALID:
      return tracker_state_t::INVALID;
    case mrs_msgs::msg::TrackerStatus::STATE_IDLE:
      return tracker_state_t::IDLE;
    case mrs_msgs::msg::TrackerStatus::STATE_TAKEOFF:
      return tracker_state_t::TAKEOFF;
    case mrs_msgs::msg::TrackerStatus::STATE_HOVER:
      return tracker_state_t::HOVER;
    case mrs_msgs::msg::TrackerStatus::STATE_REFERENCE:
      return tracker_state_t::REFERENCE;
    case mrs_msgs::msg::TrackerStatus::STATE_TRAJECTORY:
      return tracker_state_t::TRAJECTORY;
    case mrs_msgs::msg::TrackerStatus::STATE_LAND:
      return tracker_state_t::LAND;
    default:
      return tracker_state_t::UNKNOWN;
  }
}

state_t StateMonitor::parse_uav_state(mrs_msgs::msg::HwApiStatus::ConstSharedPtr               hw_api_status,
                                      mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diagnostics) {
  if (hw_api_status == nullptr || control_manager_diagnostics == nullptr)
    return state_t::UNKNOWN;

  const bool hw_armed = hw_api_status->armed;
  // not armed
  if (!hw_armed)
    return state_t::DISARMED;

  // armed, flying in manual mode
  const bool manual_mode = hw_api_status->mode == "MANUAL";
  if (control_manager_diagnostics->joystick_active && manual_mode)
    return state_t::MANUAL;

  // armed, not flying
  const auto tracker_state = parse_tracker_state(control_manager_diagnostics);
  const bool null_tracker  = tracker_state == tracker_state_t::INVALID;
  if (hw_armed && null_tracker) {
    const bool offboard = hw_api_status->offboard;
    if (offboard)
      return state_t::OFFBOARD;
    return state_t::ARMED;
  }
  // flying using the MRS system in RC joystick mode
  if (control_manager_diagnostics->joystick_active)
    return state_t::RC_MODE;

  // LandoffTracker goes into idle state when deactivating
  if (control_manager_diagnostics->active_tracker == "LandoffTracker" && tracker_state == tracker_state_t::IDLE)
    return state_t::TAKEOFF;

  // unless the RC mode is active, just parse the tracker state
  switch (tracker_state) {
    case tracker_state_t::TAKEOFF:
      return state_t::TAKEOFF;
    case tracker_state_t::HOVER:
      return state_t::HOVER;
    case tracker_state_t::REFERENCE:
      return state_t::GOTO;
    case tracker_state_t::TRAJECTORY:
      return state_t::TRAJECTORY;
    case tracker_state_t::LAND:
      return state_t::LAND;
    default:
      return state_t::UNKNOWN;
  }
}

mrs_msgs::msg::GeneralRobotInfo StateMonitor::parse_general_robot_info(sensor_msgs::msg::BatteryState::ConstSharedPtr battery_state) {
  mrs_msgs::msg::GeneralRobotInfo msg;
  msg.stamp            = clock_->now();
  msg.robot_name       = _robot_name_;
  msg.robot_type       = static_cast<int>(robot_type_);
  msg.robot_ip_address = robot_ip_address_;

  const bool autostart_running = sh_automatic_start_can_takeoff_.getNumPublishers();
  const bool autostart_ready   = sh_automatic_start_can_takeoff_.hasMsg() && sh_automatic_start_can_takeoff_.getMsg()->data;

  const auto uav_state = uav_state_.value();

  const bool state_offboard = uav_state == state_t::OFFBOARD;
  msg.ready_to_start        = state_offboard && autostart_running && autostart_ready;
  msg.problems_preventing_start.clear();

  // If not flying, check what is preventing the start and add it to the message.
  // If flying, we can assume everything was fine at takeoff, so no need to check for problems preventing start
  if (!is_flying_autonomously(uav_state)) {
    switch (uav_state) {
      case state_t::UNKNOWN:
        msg.problems_preventing_start.emplace_back("UAV state is UNKNOWN");
        break;
      case state_t::MANUAL:
        msg.problems_preventing_start.emplace_back("UAV state is in MANUAL mode");
        break;
      case state_t::DISARMED:
        msg.problems_preventing_start.emplace_back("UAV is DISARMED");
        break;
      case state_t::OFFBOARD:
        // In OFFBOARD but not flying — autostart checks below will explain why
        break;
      default:
        msg.problems_preventing_start.emplace_back("UAV is not in OFFBOARD mode");
        break;
    }

    if (state_offboard && !autostart_running)
      msg.problems_preventing_start.emplace_back("Automatic start node is not running");
    else if (state_offboard && !autostart_ready) {
      // Find the root cause of autostart not being ready
      std::scoped_lock lck(errorgraph_mtx_);
      const auto       dependency_roots = errorgraph_.find_dependency_roots(autostart_node_id_);
      if (dependency_roots.empty()) {
        msg.problems_preventing_start.emplace_back("Automatic start reports UAV not ready");
      } else {
        for (const auto &root : dependency_roots) {
          // For each root, check if it's an node error or a missing topic and add it to the problems preventing start
          std::visit(
              [&msg](const auto &info) {
                using T = std::decay_t<decltype(info)>;
                if constexpr (std::is_same_v<T, mrs_lib::errorgraph::Errorgraph::node_info_t>) {
                  // If it's a node error, add all errors of the node to the problems preventing start
                  for (const auto &error : info.errors)
                    msg.problems_preventing_start.push_back(error.type);
                } else {
                  // If it's a missing topic, add the topic name to the problems preventing start
                  msg.problems_preventing_start.push_back("waiting for topic: " + info.topic_name);
                }
              },
              root);
        }
      }
    }
  }

  { // find all errors
    std::scoped_lock lck(errorgraph_mtx_);

    const auto error_roots = errorgraph_.find_error_roots();
    for (const auto &root : error_roots) {
      std::visit(
          [&msg](const auto &info) {
            using T = std::decay_t<decltype(info)>;
            if (info.not_reporting) {
              std::stringstream ss;
              ss << info.source_node.node << "." << info.source_node.component << ": not responding";
              msg.errors.push_back(ss.str());
            }
            if constexpr (std::is_same_v<T, mrs_lib::errorgraph::Errorgraph::node_info_t>) {
              for (const auto &error : info.errors)
                msg.errors.push_back(error.type);
            }
          },
          root);
    }
  }
  return msg;
}

mrs_msgs::msg::StateEstimationInfo StateMonitor::parse_state_estimation_info(mrs_msgs::msg::EstimationDiagnostics::ConstSharedPtr estimation_diagnostics,
                                                                             mrs_msgs::msg::Float64Stamped::ConstSharedPtr        local_heading,
                                                                             sensor_msgs::msg::NavSatFix::ConstSharedPtr          global_position,
                                                                             mrs_msgs::msg::Float64Stamped::ConstSharedPtr        global_heading) {
  auto init_msg         = init_state_estimation_info();
  init_msg.header.stamp = clock_->now();

  mrs_msgs::msg::StateEstimationInfo msg = init_msg;

  const bool is_estimation_diagnostics_valid = estimation_diagnostics != nullptr;
  const bool is_local_heading_valid          = local_heading != nullptr;
  const bool is_global_position_valid        = global_position != nullptr;
  const bool is_global_heading_valid         = global_heading != nullptr;

  if (is_estimation_diagnostics_valid) {
    msg.header = estimation_diagnostics->header;

    msg.local_pose.position       = estimation_diagnostics->pose.position;
    msg.above_ground_level_height = estimation_diagnostics->agl_height;

    msg.velocity     = estimation_diagnostics->velocity;
    msg.acceleration = estimation_diagnostics->acceleration;

    if (!estimation_diagnostics->running_state_estimators.empty())
      msg.current_estimator = estimation_diagnostics->running_state_estimators.at(0);

    msg.running_estimators    = estimation_diagnostics->running_state_estimators;
    msg.switchable_estimators = estimation_diagnostics->switchable_state_estimators;
  }

  if (is_local_heading_valid)
    msg.local_pose.heading = local_heading->value;

  if (is_global_position_valid) {
    msg.global_pose.position.x = global_position->latitude;
    msg.global_pose.position.y = global_position->longitude;
    msg.global_pose.position.z = global_position->altitude;
  }

  if (is_global_heading_valid)
    msg.global_pose.heading = global_heading->value;

  return msg;
}

mrs_msgs::msg::ControlInfo StateMonitor::parse_control_info(mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr    control_manager_diagnostics,
                                                            mrs_msgs::msg::ConstraintManagerDiagnostics::ConstSharedPtr constraint_manager_diagnostics,
                                                            mrs_msgs::msg::GainManagerDiagnostics::ConstSharedPtr       gain_manager_diagnostics,
                                                            std_msgs::msg::Float64::ConstSharedPtr                      thrust) {

  mrs_msgs::msg::ControlInfo msg;

  const bool is_control_manager_diagnostics_valid    = control_manager_diagnostics != nullptr;
  const bool is_constraint_manager_diagnostics_valid = constraint_manager_diagnostics != nullptr;
  const bool is_gain_manager_diagnostics_valid       = gain_manager_diagnostics != nullptr;
  const bool is_thrust_valid                         = thrust != nullptr;

  if (is_control_manager_diagnostics_valid) {
    msg.active_controller     = control_manager_diagnostics->active_controller;
    msg.available_controllers = control_manager_diagnostics->available_controllers;
    msg.active_tracker        = control_manager_diagnostics->active_tracker;
    msg.available_trackers    = control_manager_diagnostics->available_trackers;
  }

  if (is_thrust_valid)
    msg.thrust = thrust->data;

  if (is_constraint_manager_diagnostics_valid) {
    msg.active_constraints    = constraint_manager_diagnostics->current_name;
    msg.available_constraints = constraint_manager_diagnostics->available;
  }

  if (is_gain_manager_diagnostics_valid) {
    msg.active_gains    = gain_manager_diagnostics->current_name;
    msg.available_gains = gain_manager_diagnostics->available;
  }

  return msg;
}

mrs_msgs::msg::CollisionAvoidanceInfo
StateMonitor::parse_collision_avoidance_info(mrs_msgs::msg::MpcTrackerDiagnostics::ConstSharedPtr mpc_tracker_diagnostics) {
  mrs_msgs::msg::CollisionAvoidanceInfo msg;

  const bool is_mpc_tracker_diagnostics_valid = mpc_tracker_diagnostics != nullptr;

  if (is_mpc_tracker_diagnostics_valid) {
    msg.collision_avoidance_enabled = mpc_tracker_diagnostics->collision_avoidance_active;
    msg.avoiding_collision          = mpc_tracker_diagnostics->avoiding_collision;
    msg.other_robots_visible        = mpc_tracker_diagnostics->avoidance_active_uavs;
  }

  return msg;
}

mrs_msgs::msg::UavInfo StateMonitor::parse_uav_info(mrs_msgs::msg::HwApiStatus::ConstSharedPtr hw_api_status,
                                                    mrs_msgs::msg::UavStatus::ConstSharedPtr uav_status, std_msgs::msg::Float64::ConstSharedPtr mass_nominal,
                                                    std_msgs::msg::Float64::ConstSharedPtr mass_estimate) {
  mrs_msgs::msg::UavInfo msg;

  const bool is_hw_api_status_valid = hw_api_status != nullptr;
  const bool is_uav_status_valid    = uav_status != nullptr;
  const bool is_mass_nominal_valid  = mass_nominal != nullptr;
  const bool is_mass_estimate_valid = mass_estimate != nullptr;

  if (is_hw_api_status_valid) {
    msg.armed    = hw_api_status->armed;
    msg.offboard = hw_api_status->offboard;
  }

  if (is_uav_status_valid)
    msg.flight_duration = uav_status->secs_flown;

  msg.flight_state = to_string(uav_state_.value());

  if (is_mass_nominal_valid)
    msg.mass_nominal = mass_nominal->data;

  if (is_mass_estimate_valid)
    msg.mass_estimate = mass_estimate->data;

  return msg;
}

mrs_msgs::msg::SystemHealthInfo StateMonitor::parse_system_health_info(mrs_msgs::msg::UavStatus::ConstSharedPtr        uav_status,
                                                                       sensor_msgs::msg::MagneticField::ConstSharedPtr magnetic_field,
                                                                       mrs_msgs::msg::HwApiRcRssi::ConstSharedPtr      rc_rssi) {
  mrs_msgs::msg::SystemHealthInfo msg;

  const bool is_uav_status_valid     = uav_status != nullptr;
  const bool is_magnetic_field_valid = magnetic_field != nullptr;

  if (is_uav_status_valid) {
    msg.cpu_load   = uav_status->cpu_load;
    msg.free_ram   = uav_status->free_ram;
    msg.total_ram  = uav_status->total_ram;
    msg.free_hdd   = uav_status->free_hdd;
    const size_t n = std::min(uav_status->node_cpu_loads.cpu_loads.size(), uav_status->node_cpu_loads.node_names.size());
    for (size_t it = 0; it < n; it++) {
      mrs_msgs::msg::CpuLoad node_cpu_load;
      node_cpu_load.node_name = uav_status->node_cpu_loads.node_names.at(it);
      node_cpu_load.cpu_load  = uav_status->node_cpu_loads.cpu_loads.at(it);
      msg.node_cpu_loads.push_back(node_cpu_load);
    }

    msg.hw_api_rate           = uav_status->hw_api_hz;
    msg.control_manager_rate  = uav_status->control_manager_diag_hz;
    msg.state_estimation_rate = uav_status->odom_hz;
  }

  if (is_magnetic_field_valid) {
    const Eigen::Vector3d field(magnetic_field->magnetic_field.x, magnetic_field->magnetic_field.y, magnetic_field->magnetic_field.z);
    msg.mag_strength          = field.norm();
    const Eigen::Matrix3d cov = cov2eigen(magnetic_field->magnetic_field_covariance);
    msg.mag_uncertainty       = std::cbrt(cov.determinant());
  }

  // Get Wifi info from the system
  const auto wifi = readWifiInfo();
  if (!wifi.interface.empty()) {
    msg.wifi_interface    = wifi.interface;
    msg.wifi_signal_dbm   = wifi.signal_dbm;
    msg.wifi_link_quality = wifi.link_quality;
  }

  // Get RC signal info
  if (rc_rssi != nullptr) {
    msg.rc_rssi = static_cast<float>(rc_rssi->rssi);
  }

  msg.available_sensors = available_sensors_;

  return msg;
}

// | -------------------- Msg init methods -------------------- |
mrs_msgs::msg::StateEstimationInfo StateMonitor::init_state_estimation_info() {
  mrs_msgs::msg::StateEstimationInfo msg;

  msg.header.stamp    = clock_->now();
  msg.header.frame_id = "";

  msg.local_pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.local_pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.local_pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg.local_pose.heading    = std::numeric_limits<double>::quiet_NaN();

  msg.velocity.linear.x  = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.linear.y  = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.linear.z  = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.velocity.angular.z = std::numeric_limits<double>::quiet_NaN();

  msg.acceleration.linear.x  = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.linear.y  = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.linear.z  = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.acceleration.angular.z = std::numeric_limits<double>::quiet_NaN();

  msg.above_ground_level_height = std::numeric_limits<double>::quiet_NaN();

  msg.global_pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.global_pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.global_pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg.global_pose.heading    = std::numeric_limits<double>::quiet_NaN();

  msg.current_estimator = "unknown";

  return msg;
}

StateMonitor::WifiInfo StateMonitor::readWifiInfo() {

  // return cached value if less than WIFI_READ_INTERVAL_S has passed
  const auto now = clock_->now();
  if ((now - last_wifi_read_time_).seconds() < WIFI_READ_INTERVAL_S) {
    return cached_wifi_info_;
  }
  last_wifi_read_time_ = now;

  WifiInfo info;

  std::ifstream file("/proc/net/wireless");
  if (!file.is_open()) {
    cached_wifi_info_ = info;
    return info;
  }

  std::string line;

  // skip 2 header lines
  std::getline(file, line);
  std::getline(file, line);

  while (std::getline(file, line)) {
    char  iface_buf[64] = {};
    int   status = 0, link = 0;
    float level = 0.0f, noise = 0.0f;

    if (std::sscanf(line.c_str(), "%63s %d %d. %f. %f.", iface_buf, &status, &link, &level, &noise) < 4) {
      continue;
    }

    // remove trailing colon from interface name (e.g. "wlp2s0:" -> "wlp2s0")
    std::string iface(iface_buf);
    if (!iface.empty() && iface.back() == ':') {
      iface.pop_back();
    }

    // if a specific interface is configured, only match that one
    if (!_wifi_interface_.empty() && iface != _wifi_interface_) {
      continue;
    }

    info.interface    = iface;
    info.signal_dbm   = level;
    info.link_quality = link;
    cached_wifi_info_ = info;
    return info;
  }

  cached_wifi_info_ = info;
  return info;
}

} // namespace state_monitor
} // namespace mrs_robot_diagnostics

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_robot_diagnostics::state_monitor::StateMonitor)
