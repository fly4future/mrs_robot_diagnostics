#include <mrs_robot_diagnostics/sensor_plugins/camera_handler.h>

namespace mrs_robot_diagnostics
{

namespace camera_handler
{
bool CameraHandler::onInitialize(rclcpp::Node::SharedPtr &node, [[maybe_unused]] const std::string &config_key,
                               [[maybe_unused]] const std::string &name_space, rclcpp::CallbackGroup::SharedPtr cbkgrp_subs) {

  // Initialize tf2 components
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node);

  // Declare parameters before accessing them
  if (!node->has_parameter("image_topic")) {
    node->declare_parameter("image_topic", "");
  }
  if (!node->has_parameter("camera_info_topic")) {
    node->declare_parameter("camera_info_topic", "");
  }
  if (!node->has_parameter("camera_orientation_topic")) {
    node->declare_parameter("camera_orientation_topic", "");
  }
  if (!node->has_parameter("camera_frame")) {
    node->declare_parameter("camera_frame", "");
  }
  if (!node->has_parameter("optical_frame")) {
    node->declare_parameter("optical_frame", "");
  }
  if (!node->has_parameter("fcu_frame")) {
    node->declare_parameter("fcu_frame", "");
  }

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node;
  shopts.node_name                           = "StateMonitor";
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs;


  std::string image_topic_name;
  node->get_parameter("image_topic", image_topic_name);
  topic_ = image_topic_name; // Set the main topic for status updates
  RCLCPP_INFO(node->get_logger(), " Subscribing to image topic: %s", image_topic_name.c_str());

  std::string camera_info_topic_name;
  node->get_parameter("camera_info_topic", camera_info_topic_name);
  RCLCPP_INFO(node->get_logger(), " Subscribing to camera info topic: %s", camera_info_topic_name.c_str());

  std::string camera_orientation_topic_name;
  node->get_parameter("camera_orientation_topic", camera_orientation_topic_name);
  RCLCPP_INFO(node->get_logger(), " Subscribing to camera orientation topic: %s", camera_orientation_topic_name.c_str());

  if (!node->get_parameter("camera_frame", _camera_frame_)) {
    RCLCPP_WARN(node->get_logger(), "Parameter 'camera_frame' not found, failed initialization");
    return false;
  }

  if (!node->get_parameter("optical_frame", _optical_frame_)) {
    RCLCPP_WARN(node->get_logger(), "Parameter 'optical_frame' not found, failed initialization");
    return false;
  }

  if (!node->get_parameter("fcu_frame", _fcu_frame_)) {
    RCLCPP_WARN(node->get_logger(), "Parameter 'fcu_frame' not found, failed initialization");
    return false;
  }

  // Subscribers
  RCLCPP_INFO(node->get_logger(), "Subscribing to camera info topic: %s", camera_info_topic_name.c_str());
  sh_image_              = create_main_subscriber<sensor_msgs::msg::Image>(node, image_topic_name);
  sh_camera_info_        = mrs_lib::SubscriberHandler<sensor_msgs::msg::CameraInfo>(shopts, camera_info_topic_name);
  sh_camera_orientation_ = mrs_lib::SubscriberHandler<std_msgs::msg::Float32MultiArray>(shopts, camera_orientation_topic_name);

  // Publisher
  ph_camera_details_ = mrs_lib::PublisherHandler<mrs_msgs::msg::SensorInfo>(node, "~/sensor_info_out");

  RCLCPP_INFO(node->get_logger(), "[CameraHandler] '%s' initialized, topic: '%s'", name_.c_str(), topic_.c_str());
  is_initialized_ = true;
  return true;
}

mrs_msgs::msg::SensorStatus CameraHandler::updateStatus() {
  mrs_msgs::msg::SensorStatus ss_msg;
  ss_msg.name  = name_;
  ss_msg.type  = mrs_msgs::msg::SensorStatus::TYPE_CAMERA;
  ss_msg.topic = topic_;

  if (!is_initialized_) {
    ss_msg.ready  = false;
    ss_msg.rate   = -1;
    // ss_msg.status = "NOT_INITIALIZED";
    return ss_msg;
  }

  nlohmann::json camera_info_json;
  if (sh_camera_info_.hasMsg()) {

    ss_msg.rate = measured_rate_;

    bool has_image = sh_image_.hasMsg();
    ss_msg.ready   = has_image;
    if (has_image) {
      // ss_msg.status = "OK";
    } else {
      // ss_msg.status = "NO_IMAGE_DATA";
    }

    auto         msg    = sh_camera_info_.getMsg();
    const double height = msg->height;
    const double width  = msg->width;
    const double fx     = msg->k[0];
    const double fy     = msg->k[4];

    const double fov_x = 2 * atan(width / (2 * fx));
    const double fov_y = 2 * atan(height / (2 * fy));

    camera_info_json = {
        {"height", height},
        {"width", width},
        {"fov_x_rad", fov_x},
        {"fov_y_rad", fov_y},
    };

  } else {
    ss_msg.ready  = false;
    ss_msg.rate   = -1;
    // ss_msg.status = "NO_CAMERA_INFO";
  }

  geometry_msgs::msg::TransformStamped transform;
  nlohmann::json                                 camera_tf_json;
  try {
    transform = tf_buffer_->lookupTransform(_fcu_frame_, _camera_frame_, tf2::TimePointZero);
    double x  = transform.transform.translation.x;
    double y  = transform.transform.translation.y;
    double z  = transform.transform.translation.z;

    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    double          roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    camera_tf_json = {
        {"translation", {{"x", x}, {"y", y}, {"z", z}}},
        {"rotation_rpy", {{"roll", roll}, {"pitch", pitch}, {"yaw", yaw}}},
    };
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(rclcpp::get_logger("CameraHandler"), "%s", ex.what());
  }


  nlohmann::json optical_tf_json;
  try {
    transform = tf_buffer_->lookupTransform(_fcu_frame_, _optical_frame_, tf2::TimePointZero);
    double x  = transform.transform.translation.x;
    double y  = transform.transform.translation.y;
    double z  = transform.transform.translation.z;

    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    double          roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    optical_tf_json = {
        {"translation", {{"x", x}, {"y", y}, {"z", z}}},
        {"rotation_rpy", {{"roll", roll}, {"pitch", pitch}, {"yaw", yaw}}},
    };
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(rclcpp::get_logger("CameraHandler"), "%s", ex.what());
  }


  nlohmann::json camera_orientation_json;
  if (sh_camera_orientation_.hasMsg()) {
    auto orientation_msg    = sh_camera_orientation_.getMsg();
    camera_orientation_json = {
        {"orientation_rpy",
         {
             {"roll", orientation_msg->data[0]},
             {"pitch", orientation_msg->data[1]},
             {"yaw", orientation_msg->data[2]},
         }},
    };
  }

  nlohmann::json json_msg = {
      {"camera_topic", topic_},
      {"camera_frame_tf", camera_tf_json},
      {"optical_frame_tf", optical_tf_json},
      {"camera_info", camera_info_json},
      {"camera_orientation", camera_orientation_json},
  };

  std::string json_str = json_msg.dump();

  mrs_msgs::msg::SensorInfo sensor_info_msg;
  sensor_info_msg.type    = mrs_msgs::msg::SensorStatus::TYPE_CAMERA;
  sensor_info_msg.details = json_str;
  ph_camera_details_.publish(sensor_info_msg);
  // Return the camera status
  return ss_msg;
}

} // namespace camera_handler
} // namespace mrs_robot_diagnostics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_robot_diagnostics::camera_handler::CameraHandler, mrs_robot_diagnostics::SensorHandler)
