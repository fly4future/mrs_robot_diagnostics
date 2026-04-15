#include <mrs_robot_diagnostics/sensor_handler.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <mrs_msgs/msg/sensor_info.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


namespace mrs_robot_diagnostics
{

namespace camera_handler
{

class CameraHandler : public mrs_robot_diagnostics::SensorHandler {
public:
  CameraHandler() = default;

  bool                        onInitialize(rclcpp::Node::SharedPtr &node, const std::string &config_key, const std::string &name_space,
                                           rclcpp::CallbackGroup::SharedPtr cbkgrp_subs = nullptr) override;
  mrs_msgs::msg::SensorStatus updateStatus() override;


private:
  std::string _camera_frame_;
  std::string _optical_frame_;
  std::string _fcu_frame_;


  mrs_lib::SubscriberHandler<sensor_msgs::msg::CameraInfo>     sh_camera_info_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Image>          sh_image_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float32MultiArray> sh_camera_orientation_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::SensorInfo>         ph_camera_details_;

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace camera_handler
} // namespace mrs_robot_diagnostics
