#ifndef DAI_VI_ROS2__DAI_VI_ROS2_HPP_
#define DAI_VI_ROS2__DAI_VI_ROS2_HPP_

#include "dai_vi.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realsense2_camera_msgs/msg/metadata.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <memory>
#include <string>
#include <unordered_map>

namespace dai_vi
{

  class ROS2Wrapper : public rclcpp::Node
  {
  public:
    ROS2Wrapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ROS2Wrapper();

  private:
    std::string frame_prefix;
    std::unordered_map<std::string, std::string> frame_cam;
    std::string frame_imu;
    bool exposure_in_frame_id;

    std::chrono::nanoseconds time_offset;

    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> pub_cam;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<realsense2_camera_msgs::msg::Metadata>>> pub_cam_meta;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> pub_imu;

    std::unique_ptr<dai_vi::SensorWrapper> sensor;

    void publish_img(std::shared_ptr<dai::ImgFrame> img, const std::string &name);
    void publish_imu(const dai::IMUPacket &pkt);
  };

} // namespace dai_vi

#endif // DAI_VI_ROS2__DAI_VI_ROS2_HPP_
