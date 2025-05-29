#include "dai_vi_ros2/dai_vi_ros2.hpp"

#ifndef NDEBUG
#define RCLCPP_NDEBUG(logger, fmt, ...) RCLCPP_DEBUG(logger, fmt, __VA_ARGS__)
#else
#define RCLCPP_NDEBUG(logger, fmt, ...)
#endif

namespace dai_vi
{

  ROS2Wrapper::ROS2Wrapper(const rclcpp::NodeOptions &options)
      : Node("dai_vi", options)
  {
    // Handle ROS parameters
    declare_parameter("imu_hz", 200);
    declare_parameter("cam_hz", 20);
    declare_parameter("camera_ids", std::vector<int64_t>({0, 1, 2, 3}));
    declare_parameter("pub_raw", true);
    declare_parameter("exposure", 0);

    declare_parameter("frame_imu", "imu");
    declare_parameter("cam_prefix", "cam");

    uint16_t imu_hz = get_parameter("imu_hz").as_int();
    uint16_t cam_hz = get_parameter("cam_hz").as_int();
    std::vector<int64_t> camera_ids = get_parameter("camera_ids").as_integer_array();
    bool pub_raw = get_parameter("pub_raw").as_bool();
    uint32_t exposure = get_parameter("exposure").as_int();

    frame_imu = get_parameter("frame_imu").as_string();
    cam_prefix = get_parameter("cam_prefix").as_string();

    auto ros_time = get_clock()->now();
    auto steady_time = std::chrono::steady_clock::now();
    time_offset = std::chrono::nanoseconds(ros_time.nanoseconds() - steady_time.time_since_epoch().count());

    // Setup DAI-VI sensor
    sensor = std::make_unique<dai_vi::SensorWrapper>();

    if (imu_hz > 0)
    {
      sensor->createIMU(imu_hz);
      sensor->fn_proc_imu = std::bind(&ROS2Wrapper::publish_imu, this, std::placeholders::_1);

      pub_imu = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    }

    if (!camera_ids.empty() && cam_hz > 0)
    {
      for (const auto camid : camera_ids)
      {
        const auto name = cam_prefix + std::to_string(camid);
        auto cam = sensor->createCamera(name, static_cast<dai::CameraBoardSocket>(camid));
        if (!cam)
          throw std::invalid_argument("Failed to create camera!");
        if (exposure > 0)
          cam->initialControl.setManualExposure(std::chrono::microseconds(exposure), 100);

        if (pub_raw)
          pub_cam_raw[name] = create_publisher<sensor_msgs::msg::Image>(name + "/image_raw", rclcpp::SensorDataQoS());
        else
          pub_cam_comp[name] = create_publisher<sensor_msgs::msg::CompressedImage>(name + "/image_raw/compressed", rclcpp::SensorDataQoS());
        pub_cam_info[name] = create_publisher<image_info_msgs::msg::ImageInfo>(name + "/image_info", rclcpp::SensorDataQoS());
      }
      sensor->cam_hz = cam_hz;
      sensor->encode = !pub_raw;
      if (exposure > 0)
        sensor->start_skip = 1;

      if (pub_raw)
        sensor->fn_proc_synced = std::bind(&ROS2Wrapper::publish_images<dai::ImgFrame>, this, std::placeholders::_1);
      else
        sensor->fn_proc_synced = std::bind(&ROS2Wrapper::publish_images<dai::EncodedFrame>, this, std::placeholders::_1);
    }

    if (!sensor->buildPipeline())
      throw std::invalid_argument("Failed to build pipeline!");
    if (!sensor->createDevice())
      throw std::runtime_error("Failed to create device!");
    if (!sensor->start())
      throw std::runtime_error("Failed to start sensor!");
  }

  template <class imgT, bool exact_stamp>
  void ROS2Wrapper::publish_images(std::shared_ptr<dai::MessageGroup> msgpack)
  {
    std_msgs::msg::Header header;
    if constexpr (!exact_stamp)
      header.stamp = rclcpp::Time((msgpack->getTimestamp().time_since_epoch() + time_offset).count());

    for (const auto &[name, msg] : *msgpack)
    {
      const auto &img = std::dynamic_pointer_cast<imgT>(msg);
      if constexpr (exact_stamp)
        header.stamp = rclcpp::Time((img->getTimestamp().time_since_epoch() + time_offset).count());
      header.frame_id = name + "_optical_frame";

      image_info_msgs::msg::ImageInfo img_info;
      img_info.header = header;
      img_info.exposure = rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(img->getExposureTime()).count());
      img_info.iso = img->getSensitivity();

      if constexpr (std::is_same_v<imgT, dai::ImgFrame>)
      {
        auto pub = pub_cam_raw[name];
        if (pub->get_subscription_count() > 0)
        {
          RCLCPP_NDEBUG(get_logger(), "<publish_images> stamp=%f dai=%p", rclcpp::Time(header.stamp).seconds(), img->getData().data());

          auto ros_msg = std::make_unique<sensor_msgs::msg::Image>();
          ros_msg->header = header;
          ros_msg->height = img->getHeight();
          ros_msg->width = img->getWidth();
          ros_msg->encoding = "mono8";
          ros_msg->is_bigendian = false;
          ros_msg->step = img->getWidth();
          ros_msg->data = std::move(img->getData());

          RCLCPP_NDEBUG(get_logger(), "<publish_images> stamp=%f msg=%p", rclcpp::Time(header.stamp).seconds(), ros_msg->data.data());
          pub->publish(std::move(ros_msg));
        }
      }
      if constexpr (std::is_same_v<imgT, dai::EncodedFrame>)
      {
        auto pub = pub_cam_comp[name];
        if (pub->get_subscription_count() > 0)
        {
          RCLCPP_NDEBUG(get_logger(), "<publish_images> stamp=%f dai=%p", rclcpp::Time(header.stamp).seconds(), img->getData().data());

          auto ros_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
          ros_msg->header = header;
          ros_msg->format = "jpeg";
          ros_msg->data = std::move(img->getData());

          RCLCPP_NDEBUG(get_logger(), "<publish_images> stamp=%f msg=%p", rclcpp::Time(header.stamp).seconds(), ros_msg->data.data());
          pub->publish(std::move(ros_msg));
        }
      }
      pub_cam_info[name]->publish(img_info);
    }
  }

  void ROS2Wrapper::publish_imu(const dai::IMUPacket &pkt)
  {
    const auto &accel = pkt.acceleroMeter;
    const auto &gyro = pkt.gyroscope;

    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = rclcpp::Time((accel.getTimestamp().time_since_epoch() + time_offset).count());
    msg->header.frame_id = frame_imu;

    msg->linear_acceleration.x = accel.x;
    msg->linear_acceleration.y = accel.y;
    msg->linear_acceleration.z = accel.z;

    msg->angular_velocity.x = gyro.x;
    msg->angular_velocity.y = gyro.y;
    msg->angular_velocity.z = gyro.z;

    RCLCPP_NDEBUG(get_logger(), "<publish_imu> stamp=%f address=%p", rclcpp::Time(msg->header.stamp).seconds(), static_cast<void *>(msg.get()));
    pub_imu->publish(std::move(msg));
  }

} // namespace dai_vi

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dai_vi::ROS2Wrapper)
