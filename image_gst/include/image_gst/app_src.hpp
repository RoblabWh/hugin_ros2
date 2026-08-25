#ifndef IMAGE_GST__APP_SRC_HPP_
#define IMAGE_GST__APP_SRC_HPP_

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <cstdint>
#include <optional>
#include <thread>

#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_gst {

class AppSrc : public rclcpp::Node {
public:
  explicit AppSrc(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~AppSrc() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  GstElement *pipeline_ = nullptr;
  GstElement *source_ = nullptr; // borrowed from pipeline_
  GstBus *bus_ = nullptr;
  GstCaps *caps_ = nullptr;
  std::optional<int64_t> offset_;
  int64_t last_pts_ = 0;
  std::thread bus_thread_;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void busLoop();
};

} // namespace image_gst

#endif // IMAGE_GST__APP_SRC_HPP_
