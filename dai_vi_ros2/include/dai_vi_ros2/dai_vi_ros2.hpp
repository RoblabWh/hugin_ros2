#ifndef DAI_VI_ROS2__DAI_VI_ROS2_HPP_
#define DAI_VI_ROS2__DAI_VI_ROS2_HPP_

#include "dai_vi.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "image_info_msgs/msg/image_info.hpp"

namespace dai_vi
{

  class ROS2Wrapper : public rclcpp::Node
  {
  public:
    ROS2Wrapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    std::string cam_prefix;
    std::string frame_imu;

    std::chrono::nanoseconds time_offset;

    std::map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> pub_cam_raw;
    std::map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>> pub_cam_comp;
    std::map<std::string, std::shared_ptr<rclcpp::Publisher<image_info_msgs::msg::ImageInfo>>> pub_cam_info;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> pub_imu;

    std::unique_ptr<dai_vi::SensorWrapper> sensor;

    template <class imgT, bool exact_stamp = false>
    void publish_images(std::shared_ptr<dai::MessageGroup> msgpack);
    void publish_imu(const dai::IMUPacket &pkt);
  };

} // namespace dai_vi

#endif // DAI_VI_ROS2__DAI_VI_ROS2_HPP_
