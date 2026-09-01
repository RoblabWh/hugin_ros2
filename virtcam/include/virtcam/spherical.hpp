#ifndef VIRTCAM__SPHERICAL_HPP_
#define VIRTCAM__SPHERICAL_HPP_

#include "basalt/calibration/calibration.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "opencv2/core.hpp"
#include "panoweave.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace virtcam {

class VariableSynchronizer;

class SphericalCamera : public rclcpp::Node {
public:
  SphericalCamera(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  bool initialized;
  panoweave::Stitcher stitcher;
  Sophus::SE3f extr_last;

  std::unique_ptr<VariableSynchronizer> sub_sync;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;

  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
      pre_set_param_callback_handle;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      on_set_param_callback_handle;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      post_set_param_callback_handle;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  void image_callback(
      const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> &msgs);

  void pre_set_param_callback(std::vector<rclcpp::Parameter> &params);
  rcl_interfaces::msg::SetParametersResult
  on_set_param_callback(const std::vector<rclcpp::Parameter> &params);
  void post_set_param_callback(const std::vector<rclcpp::Parameter> &params);

  void build_intrinsics();
  void build_subscriptions();
};

} // namespace virtcam

#endif // VIRTCAM__SPHERICAL_HPP_
