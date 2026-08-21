#ifndef VIRTCAM__PERSPECTIVE_HPP_
#define VIRTCAM__PERSPECTIVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/core.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "basalt/calibration/calibration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/static_transform_broadcaster.hpp>

namespace virtcam {

class PerspectiveCamera : public rclcpp::Node {
public:
  PerspectiveCamera(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  bool initialized;
  basalt::Calibration<float> in_calib;
  basalt::GenericCamera<float> out_intr;
  Sophus::SE3<float> out_extr;
  cv::UMat map, vign;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub;

  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
      pre_set_param_callback_handle;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      on_set_param_callback_handle;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      post_set_param_callback_handle;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tfs_broadcaster;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

  void pre_set_param_callback(std::vector<rclcpp::Parameter> &params);
  rcl_interfaces::msg::SetParametersResult
  on_set_param_callback(const std::vector<rclcpp::Parameter> &params);
  void post_set_param_callback(const std::vector<rclcpp::Parameter> &params);

  void build_intrinsics();
  void build_extrinsics();
  void build_map();
  void build_subscription();
};

} // namespace virtcam

#endif // VIRTCAM__PERSPECTIVE_HPP_
